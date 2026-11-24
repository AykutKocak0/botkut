#include <stdint.h>
#include <cstring>
#include "Types.h"
#include <string>
#include "Board.h"
#include "Search.h"
#include "GenerateMoves.h"
#include <limits>
#include <algorithm>
#include <iostream>
#include "TransPositionTable.h"
#include <cassert>
#include <math.h>
#include "Constants.h"
#include "TimeManagement.h"
#include "Notation.h"
#include <mutex>
#include <sstream>

using namespace Constants;

static std::mutex uciOutputMutex;

void uciPrint(const std::string& line) {
    std::lock_guard<std::mutex> lock(uciOutputMutex);
    std::cout << line << std::endl;
}

void Search::startAsync(const Board& board, const SearchLimits& limits) {
    stopAndWait();
    // Reset before the thread starts, so a "stop" that arrives right after "go" is not lost.
    stopped = false;
    infinite = limits.infinite;
    // The thread gets its own (non-const) copy of the board to make moves on.
    worker = std::thread([this, boardCopy = Board(board), limits]() mutable {
        startSearch(boardCopy, limits);
    });
}

void Search::stopAndWait() {
    stopped = true;
    if (worker.joinable()) worker.join();
}

void Search::waitForFinish() {
    if (worker.joinable()) worker.join();
}

SearchResult Search::searchSync(Board& board, const SearchLimits& limits) {
    stopAndWait();
    stopped = false;
    infinite = false;
    silent = true;
    startSearch(board, limits);
    silent = false;

    SearchResult result;
    result.hasMove = hasRootMove;
    result.move = bestMoveFound;
    result.score = bestScoreFound;
    result.isMate = std::abs(bestScoreFound) > MATE_BOUND;
    return result;
}

void Search::clearHistory() {
    stopAndWait();
    memset(historyTable, 0, sizeof(historyTable));
    std::fill(contHist.begin(), contHist.end(), 0);
    std::fill(pawnCorr.begin(), pawnCorr.end(), 0);
    for (int p = 0; p < 12; p++) {
        for (int sq = 0; sq < 64; sq++) counterMoves[p][sq] = Move();
    }
}

void Search::startSearch(Board& board, const SearchLimits& limits) {
    const int maxDepth = limits.depth;
    nodes = 0;
    nodeLimit = limits.nodes;
    hasRootMove = false;
    bestScoreFound = 0;
    startTime = std::chrono::high_resolution_clock::now();
    int phase = board.getPhase();
    // "go infinite" ignores any clock values; "movetime" wins over the clock.
    bool isTimeLeft = !limits.infinite && (limits.moveTime > 0 || limits.timeLeft > 0);
    if (isTimeLeft) {
        if (limits.moveTime > 0) timeManager.startFixed(limits.moveTime);
        else timeManager.startTimer(limits.timeLeft, limits.increment, limits.movesToGo, phase);
    }
    // UCI: in infinite mode bestmove may only be sent after "stop".
    auto printBestMove = [this](const std::string& move) {
        if (silent) return;
        while (infinite && !stopped) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        uciPrint("bestmove " + move);
    };

    // History and countermoves carry over from the previous move of the game (halved,
    // so old information fades); "ucinewgame" clears them via clearHistory().
    for (auto& side : historyTable)
        for (auto& from : side)
            for (int& entry : from) entry /= 2;
    for (int& entry : contHist) entry /= 2;
    for (int i = 0; i < MAX_PLY; i++) {
        killerMoves[i][0] = Move();
        killerMoves[i][1] = Move();
        stackMoves[i] = Move();
        stackPieces[i] = -1;
    }

    timed = isTimeLeft;
    // The clock may only stop the search once depth 1 has finished. Depth 1 takes
    // well under a millisecond, but if the process is paused (other games, the OS)
    // the time can run out before it completes, and the engine used to play an
    // unsearched move then.
    rootDepthDone = false;
    TT.newSearch();
    betaCutoffs = 0;
    firstMoveCutoffs = 0;

    Move pseudoMoves[256];
    int pseudoCount = gen.generateMoves(board, pseudoMoves);

    // Keep only legal root moves so mate/stalemate is detected up front.
    Move rootMoveList[256];
    int moveCount = 0;
    for (int i = 0; i < pseudoCount; i++) {
        if (board.makeMove(pseudoMoves[i])) {
            board.unmakeMove(pseudoMoves[i]);
            rootMoveList[moveCount++] = pseudoMoves[i];
        }
    }
    if (moveCount == 0) {
        printBestMove("0000");
        return;
    }
    int fallback = 0;
    for (int i = 0; i < moveCount; i++) {
        rootMoveList[i].score = scoreMove(board, rootMoveList[i], 0);
        if (rootMoveList[i].score > rootMoveList[fallback].score) fallback = i;
    }
    // Fallback if a "stop" or node limit ends the search before depth 1 completes:
    // the best move by move-ordering score (e.g. a winning capture), not simply
    // the first move generated.
    hasRootMove = true;
    bestMoveFound = rootMoveList[fallback];
    int previousBestValue = 0;
    // Principal variation of the best root move in the current iteration.
    Move rootPV[65];
    int rootPVLength = 0;
    for (int currentDepth = 1; currentDepth <= maxDepth; currentDepth++) {
        // Past the soft limit the next iteration would most likely not finish.
        if (isTimeLeft && rootDepthDone && timeManager.isSoftTimeUp()) break;
        int alpha = -1000000;
        int beta  =  1000000;
        
        if (currentDepth >= 3) {
            alpha = previousBestValue - 50; 
            beta  = previousBestValue + 50;
        }

        int bestValue = -1000000;
        int score;
        Move currentBestMove; 
        bool hasRootHash = (currentDepth > 1); 
        bool isEndGame = board.isEndGame();
        bool wasChecked = board.isInCheck();
        if (hasRootHash) {
            for (int i = 0; i < moveCount; i++) {
                if (rootMoveList[i] == bestMoveFound) {
                    rootMoveList[i].score = SCORE_HASH;
                    break; 
                }
            }
        }
        int movesSearched = 0;
        for (int i = 0; i < moveCount; i++) {
            pickBestMove(rootMoveList, moveCount, i);
            stackMoves[0] = rootMoveList[i];
            stackPieces[0] = board.getPieceType(rootMoveList[i].from);
            if (!board.makeMove(rootMoveList[i])) continue;

            if (movesSearched == 0) {
                score = -alphaBeta(board, -beta, -alpha, currentDepth - 1, 1, true);
                if ((score <= alpha || score >= beta) && abs(score) < MATE_SCORE) {
                
                alpha = -1000000;
                beta  =  1000000;
                
                score = -alphaBeta(board, -beta, -alpha, currentDepth - 1, 1, true);
            }
            } else {
                bool isKiller = (rootMoveList[i] == killerMoves[0][0] || rootMoveList[i] == killerMoves[0][1]);
                int reduction = 0;
                if (currentDepth >= 3 && i >= 4 && !rootMoveList[i].isCapture() && !rootMoveList[i].isPromotion() && !wasChecked && !isKiller) {
                    reduction = reductionTable[std::min(currentDepth, 63)][std::min(i, 63)];
                    reduction = std::max(0, std::min(reduction, currentDepth - 2));
                    if(isEndGame){
                        reduction = (reduction == 0) ? reduction : reduction-1;
                    }
                }
                
                score = -alphaBeta(board, -alpha - 1, -alpha, currentDepth - 1 - reduction, 1, true);

                if (score > alpha) {
                    if (reduction > 0) {
                        score = -alphaBeta(board, -alpha - 1, -alpha, currentDepth - 1, 1, true);
                    }
                    if (score > alpha) {
                        // Beating the aspiration window's upper bound only proves a lower
                        // bound; open the window so this move gets an exact score and PV.
                        if (score >= beta) beta = 1000000;
                        score = -alphaBeta(board, -beta, -alpha, currentDepth - 1, 1, true);
                    }
                }
            }
            board.unmakeMove(rootMoveList[i]);
            movesSearched++;

            if (stopped) break;

            if (score > -MATE_SCORE && score < MATE_SCORE) {
                rootMoveList[i].score = score; 
            }

            if (score > bestValue) {
                bestValue = score;
                currentBestMove = rootMoveList[i];
                // The child at ply 1 left its line in pvTable[1]; prepend the root move.
                rootPV[0] = rootMoveList[i];
                rootPVLength = 1;
                for (int p = 1; p < pvLength[1]; p++) rootPV[rootPVLength++] = pvTable[1][p];
            }
            alpha = std::max(alpha, score);
        }

        if (!stopped) {
            rootDepthDone = true;
            bestMoveFound = currentBestMove;
            previousBestValue = bestValue;
            bestScoreFound = bestValue;

            auto now = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = now - startTime;
            uint64_t nps = (elapsed.count() > 0.001) ? (uint64_t)(nodes / elapsed.count()) : 0;

            std::ostringstream info;
            info << "info depth " << currentDepth
                 << " score " << scoreToUci(bestValue)
                 << " nodes " << nodes
                 << " nps " << nps
                 << " time " << (int)(elapsed.count() * 1000)
                 << " pv";
            for (int p = 0; p < rootPVLength; p++) info << " " << Notation::moveToUci(rootPV[p]);
            if (!silent) uciPrint(info.str());
            if (bestValue > MATE_BOUND) { 

                int mateDistance = MATE_SCORE - bestValue; 
                if (mateDistance <= currentDepth) {
                    break;
                }
            }
        } else {
            break; 
        }
    }

    // Move-ordering quality: share of beta cutoffs that happened on the first move tried.
    double firstMoveRate = betaCutoffs ? (100.0 * firstMoveCutoffs / betaCutoffs) : 0.0;
    std::ostringstream stats;
    stats << "info string cutoffs " << betaCutoffs
          << " first-move " << firstMoveCutoffs
          << " rate " << firstMoveRate << "%";
    if (!silent) uciPrint(stats.str());

    printBestMove(Notation::moveToUci(bestMoveFound));
}

// ---------------------------------------------------------------------------
// alphaBeta and its parts. The main function reads top to bottom as the order in
// which a node is handled; each step lives in a small helper below it.
// ---------------------------------------------------------------------------

int Search::alphaBeta(Board& board, int alpha, int beta, int depthLeft, int ply, bool allowNull) {
    nodes++;
    // Empty PV until a move raises alpha; must happen before any early return.
    pvLength[ply] = ply;
    checkLimits();

    if (ply > 0 && (board.isRepetition() || board.getMovesWH() >= 100)) return 0;
    if (ply >= MAX_PLY) return board.evaluate();

    const int originalAlpha = alpha;
    const uint64_t key = board.getZobristKey();
    if (board.isInCheck()) depthLeft++;                 // check extension
    if (stopped) return 0;
    if (depthLeft <= 0) return quiescence(board, alpha, beta, ply, 0);

    // 1. Transposition table.
    Move hashMove;
    int ttScore;
    if (probeTT(key, depthLeft, alpha, beta, ply, hashMove, ttScore)) return ttScore;
    const bool hasHashMove = hashMove.from != hashMove.to;   // fail-low entries carry a1a1

    // 2. Node-level pruning from the static evaluation (non-PV, not in check).
    const bool inCheck = board.isInCheck();
    // Zero-window nodes only prove a bound; PV nodes need an exact score and are
    // never pruned by the static-eval tricks below.
    const bool pvNode = beta - alpha > 1;
    // rawEval is the evaluation itself; staticEval adds what correction history has
    // learned about how far the evaluation is off in similar positions.
    const int rawEval = inCheck ? 0 : board.evaluate();
    const int staticEval = inCheck ? 0 : correctedEval(board, rawEval);
    int pruned;
    if (reverseFutility(pvNode, inCheck, depthLeft, beta, staticEval)) return staticEval;
    if (razor(board, pvNode, inCheck, depthLeft, alpha, ply, staticEval, pruned)) return pruned;
    if (nullMovePrune(board, inCheck, depthLeft, beta, ply, allowNull, staticEval, pruned)) return pruned;

    // Internal iterative reduction: without a hash move the ordering is weak,
    // so search this node one ply shallower and let the TT fill in for next time.
    if (!hasHashMove && depthLeft >= 4) depthLeft--;

    // 3. Move loop.
    Move moveList[256];
    const int moveCount = gen.generateMoves(board, moveList);
    scoreMoves(board, moveList, moveCount, hashMove, hasHashMove, ply);

    Move bestMove;
    int bestValue = -1000000;
    Move quietsTried[64];
    int quietsTriedCount = 0;
    int movesSearched = 0;
    int quietsSeen = 0;
    const int lmpLimit = LMP_BASE + depthLeft * depthLeft;

    for (int i = 0; i < moveCount; ++i) {
        pickBestMove(moveList, moveCount, i);
        const Move move = moveList[i];
        const bool quiet = !move.isCapture() && !move.isPromotion();

        if (skipMove(board, move, quiet, pvNode, inCheck, depthLeft, movesSearched,
                     quietsSeen, lmpLimit, bestValue)) continue;

        stackMoves[ply] = move;
        stackPieces[ply] = board.getPieceType(move.from);
        if (!board.makeMove(move)) continue;
        if (quiet) quietsSeen++;

        const int score = searchMove(board, move, i, movesSearched, alpha, beta,
                                     depthLeft, ply, inCheck);
        board.unmakeMove(move);
        // An aborted search returns meaningless scores; never let them reach the TT.
        if (stopped) return 0;
        movesSearched++;

        if (score > bestValue) {
            bestMove = move;
            bestValue = score;
        }
        if (score >= beta) {
            betaCutoffs++;
            if (movesSearched == 1) firstMoveCutoffs++;
            if (quiet) onQuietCutoff(board, move, depthLeft, ply, quietsTried, quietsTriedCount);
            if (!inCheck && quiet) updateCorrection(board, depthLeft, score, staticEval, HASH_BETA);
            TT.store(key, depthLeft, scoreToTT(score, ply), HASH_BETA, move);
            return score;
        }
        if (quiet && quietsTriedCount < 64) quietsTried[quietsTriedCount++] = move;
        if (score > alpha) {
            alpha = score;
            updatePV(ply, move);
        }
    }

    // 4. No legal move: mate or stalemate.
    if (movesSearched == 0) return board.isInCheck() ? -MATE_SCORE + ply : 0;

    // 5. Store the result.
    HashFlag flag = HASH_ALPHA;
    if (bestValue >= beta) flag = HASH_BETA;
    else if (bestValue > originalAlpha) flag = HASH_EXACT;
    // A capture as the best move says more about the tactics than about the
    // evaluation, so only quiet best moves (or a fail low) train the correction.
    const bool quietBest = !bestMove.isCapture() && !bestMove.isPromotion();
    if (!inCheck && (flag == HASH_ALPHA || quietBest))
        updateCorrection(board, depthLeft, bestValue, staticEval, flag);
    // After a fail low the "best" move is just the highest upper bound, which says
    // nothing; store no move so a good one already in the entry is kept.
    TT.store(key, depthLeft, scoreToTT(bestValue, ply), flag, flag == HASH_ALPHA ? Move() : bestMove);
    return bestValue;
}

// ---------------------------------------------------------------------------
// Correction history. Each table maps a feature of the position (here: the pawn
// structure, per side to move) to a running average of how far the search result
// was from the static evaluation. Entries are stored in 1/CORR_GRAIN centipawns.
// ---------------------------------------------------------------------------

int Search::correctedEval(const Board& board, int rawEval) {
    const int side = board.sideToMove;
    const int correction = pawnCorr[side * CORR_SIZE + board.getPawnKey() % CORR_SIZE] / CORR_GRAIN;
    return std::clamp(rawEval + correction, -MATE_BOUND + 1, MATE_BOUND - 1);
}

// Learns from the residual (bestValue - staticEval), where staticEval is the already
// corrected evaluation: the entry is nudged by the error that is still left. The
// leaves of the search (quiescence stand-pat) use the corrected evaluation too, so
// the search result already contains the correction; learning toward
// (bestValue - rawEval) would make the table feed on its own value and keep any bias
// it once picked up. With the residual, learning stops once the error is gone.
// A bound only counts in the direction it proves: a fail high (lower bound) can only
// say the evaluation was too low, a fail low (upper bound) only that it was too high.
// Shallow nodes (depth < CORR_MIN_DEPTH) do not train: measured on real games, they
// gave 72% of all updates and their differences were mostly tactics, not evaluation
// error, which filled the tables with noise.
void Search::updateCorrection(const Board& board, int depthLeft, int bestValue, int staticEval,
                              int flag) {
    if (depthLeft < CORR_MIN_DEPTH) return;
    if (std::abs(bestValue) >= MATE_BOUND) return;
    if (flag == HASH_BETA && bestValue <= staticEval) return;
    if (flag == HASH_ALPHA && bestValue >= staticEval) return;

    const int residual = std::clamp((bestValue - staticEval) * CORR_GRAIN, -CORR_MAX, CORR_MAX);
    const int weight = std::min(depthLeft + 1, 16);
    auto apply = [residual, weight](int& entry) {
        entry = std::clamp(entry + residual * weight / CORR_WEIGHT_SCALE, -CORR_MAX, CORR_MAX);
    };
    const int side = board.sideToMove;
    apply(pawnCorr[side * CORR_SIZE + board.getPawnKey() % CORR_SIZE]);
}

// Stops the search when the time or node budget is used up.
void Search::checkLimits() {
    if (timed && rootDepthDone && (nodes & 2047) == 0 && timeManager.isTimeUp()) stopped = true;
    if (nodeLimit && nodes >= nodeLimit) stopped = true;
}

// Probes the TT. Returns true with the score in `score` if the entry is deep
// enough to cut this node off; always fills `hashMove` (a1a1 when there is none).
bool Search::probeTT(uint64_t key, int depthLeft, int alpha, int beta, int ply,
                     Move& hashMove, int& score) {
    hashMove = Move();
    TTEntry* entry = TT.probe(key);
    if (entry == nullptr) return false;
    hashMove = entry->bestMove;
    if (entry->depth < depthLeft) return false;
    score = scoreFromTT(entry->score, ply);
    if (entry->flag == HASH_EXACT) return true;
    if (entry->flag == HASH_ALPHA && score <= alpha) return true;
    if (entry->flag == HASH_BETA && score >= beta) return true;
    return false;
}

// Reverse futility pruning: we are so far above beta that even after giving up a
// margin per remaining ply the opponent should not get back below it.
bool Search::reverseFutility(bool pvNode, bool inCheck, int depthLeft, int beta, int staticEval) const {
    return !pvNode && !inCheck && depthLeft <= RFP_MAX_DEPTH && std::abs(beta) < MATE_BOUND
        && staticEval - RFP_MARGIN * depthLeft >= beta;
}

// Razoring: hopeless shallow non-PV nodes drop straight into quiescence. Returns
// true with the quiescence score in `value` if that confirms the fail low.
bool Search::razor(Board& board, bool pvNode, bool inCheck, int depthLeft, int alpha,
                   int ply, int staticEval, int& value) {
    if (pvNode || inCheck || depthLeft > 2) return false;
    const int margin = 300 + 150 * depthLeft * depthLeft;
    if (staticEval >= alpha - margin) return false;
    value = quiescence(board, alpha - 1, alpha, ply, 0);
    return value < alpha && std::abs(value) < MATE_BOUND;
}

// Null move pruning, only where we already stand at or above beta: below it,
// passing almost never produces a cutoff and the reduced search is wasted.
// Returns true with the score to return in `value` (0 if the search was stopped).
bool Search::nullMovePrune(Board& board, bool inCheck, int depthLeft, int beta, int ply,
                           bool allowNull, int staticEval, int& value) {
    if (inCheck || board.isZugzwang() || depthLeft < 3 || !allowNull || staticEval < beta) return false;
    const int nullReduction = 3;
    stackPieces[ply] = -1;
    board.makeNullMove();
    const int nullScore = -alphaBeta(board, -beta, -beta + 1, depthLeft - 1 - nullReduction, ply + 1, false);
    board.unmakeNullMove();
    if (stopped) { value = 0; return true; }
    if (nullScore < beta) return false;
    // A mate found after passing is not a real mate for this position.
    value = nullScore >= MATE_BOUND ? beta : nullScore;
    return true;
}

// Ordering scores for all moves of a node; the hash move goes first.
void Search::scoreMoves(Board& board, Move* moveList, int moveCount, const Move& hashMove,
                        bool hasHashMove, int ply) {
    for (int i = 0; i < moveCount; i++) {
        moveList[i].score = (hasHashMove && moveList[i] == hashMove)
                          ? SCORE_HASH : scoreMove(board, moveList[i], ply);
    }
}

// Move-level pruning before the move is made: late move pruning and SEE pruning.
bool Search::skipMove(Board& board, const Move& move, bool quiet, bool pvNode, bool inCheck,
                      int depthLeft, int movesSearched, int quietsSeen, int lmpLimit,
                      int bestValue) const {
    // bestValue > -MATE_BOUND: some move was searched and it does not lose to a mate.
    const bool safeToPrune = !pvNode && !inCheck && bestValue > -MATE_BOUND;
    // Late move pruning: at shallow nodes, once enough quiet moves have been tried,
    // the remaining ones (ordered last) are skipped without being searched.
    if (safeToPrune && quiet && depthLeft <= LMP_MAX_DEPTH && quietsSeen >= lmpLimit) return true;
    // SEE pruning: skip moves that lose too much material on their target square
    // (the allowed loss grows with depth).
    if (safeToPrune && movesSearched > 0 && depthLeft <= SEE_PRUNE_MAX_DEPTH && !move.isPromotion()) {
        const int threshold = quiet ? -SEE_QUIET_MARGIN * depthLeft * depthLeft
                                    : -SEE_CAPTURE_MARGIN * depthLeft;
        if (!board.seeGE(move, threshold)) return true;
    }
    return false;
}

// Late move reduction for the move at index i of the ordered list.
int Search::lmrReduction(const Move& move, int i, int depthLeft, int ply, bool inCheck) const {
    const bool isKiller = move == killerMoves[ply][0] || move == killerMoves[ply][1];
    if (depthLeft < 3 || i < 4 || move.isCapture() || move.isPromotion() || inCheck || isKiller) return 0;
    const int reduction = reductionTable[std::min(depthLeft, 63)][std::min(i, 63)];
    return std::max(0, std::min(reduction, depthLeft - 2));
}

// Principal variation search of one (already made) move: the first move gets the
// full window, later ones a reduced zero-window search with re-searches.
int Search::searchMove(Board& board, const Move& move, int i, int movesSearched, int alpha,
                       int beta, int depthLeft, int ply, bool inCheck) {
    if (movesSearched == 0) return -alphaBeta(board, -beta, -alpha, depthLeft - 1, ply + 1, true);

    const int reduction = lmrReduction(move, i, depthLeft, ply, inCheck);
    int score = -alphaBeta(board, -alpha - 1, -alpha, depthLeft - 1 - reduction, ply + 1, true);
    if (score > alpha && reduction > 0)
        score = -alphaBeta(board, -alpha - 1, -alpha, depthLeft - 1, ply + 1, true);
    if (score > alpha && score < beta)
        score = -alphaBeta(board, -beta, -alpha, depthLeft - 1, ply + 1, true);
    return score;
}

// A quiet move caused a beta cutoff: killers, history (with malus for the quiets
// tried before it) and the countermove of the previous move.
void Search::onQuietCutoff(Board& board, const Move& move, int depthLeft, int ply,
                           const Move* quietsTried, int quietsTriedCount) {
    if (move != killerMoves[ply][0]) {
        killerMoves[ply][1] = killerMoves[ply][0];
        killerMoves[ply][0] = move;
    }
    const int bonus = std::min(depthLeft * depthLeft, 400);
    updateHistory(board, move, ply, bonus);
    for (int q = 0; q < quietsTriedCount; q++) updateHistory(board, quietsTried[q], ply, -bonus);
    if (ply >= 1 && stackPieces[ply - 1] >= 0) {
        counterMoves[stackPieces[ply - 1]][stackMoves[ply - 1].to] = move;
    }
}

// New best line at this ply: the move followed by the child's PV.
void Search::updatePV(int ply, const Move& move) {
    pvTable[ply][ply] = move;
    for (int p = ply + 1; p < pvLength[ply + 1]; p++) pvTable[ply][p] = pvTable[ply + 1][p];
    pvLength[ply] = pvLength[ply + 1];
}

int Search::quiescence(Board& board, int alpha, int beta, int ply, int qsDepth) {
    nodes++;
    if (ply >= MAX_PLY) return board.evaluate();

    // Any TT entry is deep enough for quiescence (depth 0).
    const int originalAlpha = alpha;
    const uint64_t key = board.getZobristKey();
    TTEntry* entry = TT.probe(key);
    Move ttMove;
    if (entry != nullptr) {
        int ttScore = scoreFromTT(entry->score, ply);
        if (entry->flag == HASH_EXACT
            || (entry->flag == HASH_ALPHA && ttScore <= alpha)
            || (entry->flag == HASH_BETA && ttScore >= beta)) return ttScore;
        ttMove = entry->bestMove;
    }
    // Results are stored at depth 0, but never over a real search of this position.
    const bool mayStore = entry == nullptr || entry->depth <= 0;

    bool inCheck = board.isInCheck();
    if (!inCheck) {
        // The leaves use the corrected evaluation as well; this is where correction
        // history changes the scores the search backs up, not only pruning decisions.
        int standPat = correctedEval(board, board.evaluate());
        if (standPat >= beta) {
            if (mayStore) TT.store(key, 0, scoreToTT(standPat, ply), HASH_BETA, Move());
            return standPat;
        }
        if (standPat > alpha) alpha = standPat;
    }

    Move moveList[256];
    int moveCount = 0;

    if (inCheck) {
        moveCount = gen.generateMoves(board, moveList);
    } else {
        moveCount = gen.generateCaptureMoves(board, moveList, board.sideToMove);
        
        if (qsDepth < QS_CHECK_PLIES) {
            moveCount = gen.generateQuietCheckMoves(board, moveList, board.sideToMove, moveCount);
        }
    }

    int legalMovesSearched = 0;

    for (int i = 0; i < moveCount; i++) {
        if (inCheck) {
            moveList[i].score = scoreMove(board, moveList[i], ply);
        } else {
            if (moveList[i].isCapture() || moveList[i].isCapturePromotion()) {
                bool losing = false;
                moveList[i].score = scoreCapture(board, moveList[i], &losing);
                // Captures that lose material (SEE < 0) are not searched at all.
                if (losing) moveList[i].score = QS_SKIP;
            } else {
                // Quiet checks go after every SEE-passing capture (those score >= 100).
                moveList[i].score = 0;
            }
        }
        if (moveList[i] == ttMove) moveList[i].score = SCORE_HASH;
    }
    Move bestMove;
    for (int i = 0; i < moveCount; i++) {
        pickBestMove(moveList, moveCount, i);
        // QS_SKIP is the lowest score, so every move from here on is a losing capture.
        if (!inCheck && moveList[i].score == QS_SKIP) break;
        stackMoves[ply] = moveList[i];
        stackPieces[ply] = board.getPieceType(moveList[i].from);
        if (!board.makeMove(moveList[i])) continue;
        legalMovesSearched++;

        int score = -quiescence(board, -beta, -alpha, ply + 1, qsDepth + 1);
        
        board.unmakeMove(moveList[i]);

        if (score >= beta) {
            if (mayStore) TT.store(key, 0, scoreToTT(score, ply), HASH_BETA, moveList[i]);
            return score;
        }
        if (score > alpha) {
            alpha = score;
            bestMove = moveList[i];
        }
    }

    if (inCheck && legalMovesSearched == 0) {
        return -MATE_SCORE + ply;
    }
    if (mayStore) {
        HashFlag flag = alpha > originalAlpha ? HASH_EXACT : HASH_ALPHA;
        TT.store(key, 0, scoreToTT(alpha, ply), flag, bestMove);
    }
    return alpha;
}

int Search::scoreCapture(Board& board, Move& move, bool* losing) {
    int pieceMoved = board.getPieceType(move.from);
    int pieceTaken = board.getPieceType(move.to);

    if (move.flag == MoveFlag::EN_PASSANT) {
        pieceTaken = (board.sideToMove == WHITE) ? B_PAWN : W_PAWN;
    }

    if (losing) *losing = false;
    int score = (PIECE_VALUES[pieceTaken] * 10) - PIECE_VALUES[pieceMoved];
    // En passant is pawn-for-pawn at worst and its target square is empty, so SEE does not apply.
    if (move.flag == MoveFlag::EN_PASSANT) return score;
    // A capture can only lose material when the attacker is worth more than the victim.
    if (PIECE_VALUES[pieceMoved] > PIECE_VALUES[pieceTaken]) {
        if (!board.seeGE(move, 0)) {
            score -= 5000;
            if (losing) *losing = true;
        }
    }

    return score;
}

// Ordering bands, highest first:
//   hash move > good captures / queen promotions > killers > quiets (history)
//   > losing captures > underpromotions
int Search::scoreMove(Board& board, Move& move, int ply) {
    bool queenPromo = (move.flag == MoveFlag::PROMOTION_QUEEN || move.flag == MoveFlag::CPROMOTION_QUEEN);
    if (move.isPromotion() && !queenPromo) {
        return SCORE_UNDERPROMO;
    }

    if (move.isCapture() || move.isCapturePromotion()) {
        bool losing = false;
        int captureScore = scoreCapture(board, move, &losing);
        if (losing) return SCORE_BAD_CAPTURE + captureScore;
        return SCORE_GOOD_CAPTURE + captureScore + (queenPromo ? 9000 : 0);
    }

    if (queenPromo) {
        return SCORE_GOOD_CAPTURE + 9000;
    }

    if (killerMoves[ply][0] == move) return SCORE_KILLER_1;
    if (killerMoves[ply][1] == move) return SCORE_KILLER_2;

    // Quiet move: butterfly history plus continuation history of the last two plies.
    int piece = board.getPieceType(move.from);
    int score = historyTable[board.sideToMove][move.from][move.to];
    if (ply >= 1 && stackPieces[ply - 1] >= 0) {
        int prevPiece = stackPieces[ply - 1];
        int prevTo = stackMoves[ply - 1].to;
        if (counterMoves[prevPiece][prevTo] == move) return SCORE_COUNTER;
        score += contHistEntry(prevPiece, prevTo, piece, move.to);
    }
    if (ply >= 2 && stackPieces[ply - 2] >= 0) {
        score += contHistEntry(stackPieces[ply - 2], stackMoves[ply - 2].to, piece, move.to);
    }
    return score;
}

// Updates butterfly and continuation history for a quiet move at this ply.
// Gravity keeps each table within +/-HISTORY_MAX for both bonus and malus.
// Must be called with the move unmade (board at the node's own position).
void Search::updateHistory(const Board& board, const Move& move, int ply, int bonus) {
    auto apply = [bonus](int& entry) {
        entry += bonus - entry * std::abs(bonus) / HISTORY_MAX;
    };
    apply(historyTable[board.sideToMove][move.from][move.to]);

    int piece = board.getPieceType(move.from);
    for (int back = 1; back <= 2; back++) {
        int p = ply - back;
        if (p >= 0 && stackPieces[p] >= 0) {
            apply(contHistEntry(stackPieces[p], stackMoves[p].to, piece, move.to));
        }
    }
}

void Search::pickBestMove(Move* moveList, int moveCount, int currentIndex) {
    int bestScore = moveList[currentIndex].score;
    int bestIndex = currentIndex;

    for (int i = currentIndex + 1; i < moveCount; i++) {
        if (moveList[i].score > bestScore) {
            bestScore = moveList[i].score;
            bestIndex = i;
        }
    }

    Move temp = moveList[currentIndex];
    moveList[currentIndex] = moveList[bestIndex];
    moveList[bestIndex] = temp;
}


// UCI wants mate distance in moves, not plies: positive = we mate, negative = we get mated.
std::string Search::scoreToUci(int score) {
    if (score > MATE_BOUND) {
        int plies = MATE_SCORE - score;           // 89999 -> mate in 1 ply
        return "mate " + std::to_string((plies + 1) / 2);
    }
    if (score < -MATE_BOUND) {
        int plies = MATE_SCORE + score;           // -89998 -> mated in 2 plies
        return "mate -" + std::to_string(plies / 2);
    }
    return "cp " + std::to_string(score);
}

