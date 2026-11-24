#pragma once
#include <stdint.h>
#include "Types.h"
#include <string>
#include "Board.h"
#include <atomic>
#include "GenerateMoves.h"
#include <chrono>
#include <vector>
#include <thread>
#include "TimeManagement.h"

// Prints one line of UCI output. Safe to call from the search thread and the
// input thread at the same time (lines never interleave).
void uciPrint(const std::string& line);

// What a "go" command asked for.
struct SearchLimits {
    int depth = 64;
    int timeLeft = 0;    // our clock in ms (wtime/btime), 0 = not given
    int increment = 0;
    int movesToGo = 0;
    int moveTime = 0;    // fixed ms per move, 0 = not given
    uint64_t nodes = 0;  // stop after this many nodes, 0 = no limit
    bool infinite = false;
};

// Outcome of a synchronous search (see Search::searchSync).
struct SearchResult {
    bool hasMove = false;  // false if the side to move has no legal move
    Move move;
    int score = 0;         // from the side to move's point of view
    bool isMate = false;   // score is a mate score
};

class Search {
public:
    ~Search() { stopAndWait(); }

    // Searches on the calling thread without printing anything and returns the
    // result. Used by datagen; must not be mixed with a running async search.
    SearchResult searchSync(Board& board, const SearchLimits& limits);

    // Starts searching a copy of the board on a background thread and returns
    // immediately. A search that is still running is stopped first.
    void startAsync(const Board& board, const SearchLimits& limits);
    // Asks the running search to stop and waits until it has printed bestmove.
    void stopAndWait();
    // Waits for the running search to finish on its own.
    void waitForFinish();
    // Forgets history, continuation history and countermoves (new game). Searches
    // otherwise keep them, halved, from one move to the next.
    void clearHistory();
    bool isInfinite() const { return infinite; }

    Move getBestMove() const { return bestMoveFound; }
    uint64_t getNodes() const { return nodes; }

private:
    void startSearch(Board& board, const SearchLimits& limits);

    std::atomic<bool> stopped{false};
    std::atomic<bool> infinite{false};
    std::thread worker;
    bool silent = false;         // suppress info/bestmove output (searchSync)
    uint64_t nodeLimit = 0;      // 0 = none
    bool hasRootMove = false;
    int bestScoreFound = 0;      // score of the last completed iteration

    int alphaBeta(Board& board, int alpha, int beta, int depthLeft, int ply, bool allowNull);
    int quiescence(Board& board, int alpha, int beta, int ply, int qsDepth = 0);

    // Parts of alphaBeta, in the order a node uses them (see Search.cpp).
    void checkLimits();
    bool probeTT(uint64_t key, int depthLeft, int alpha, int beta, int ply,
                 Move& hashMove, int& score);
    bool reverseFutility(bool pvNode, bool inCheck, int depthLeft, int beta, int staticEval) const;
    bool razor(Board& board, bool pvNode, bool inCheck, int depthLeft, int alpha,
               int ply, int staticEval, int& value);
    bool nullMovePrune(Board& board, bool inCheck, int depthLeft, int beta, int ply,
                       bool allowNull, int staticEval, int& value);
    void scoreMoves(Board& board, Move* moveList, int moveCount, const Move& hashMove,
                    bool hasHashMove, int ply);
    bool skipMove(Board& board, const Move& move, bool quiet, bool pvNode, bool inCheck,
                  int depthLeft, int movesSearched, int quietsSeen, int lmpLimit,
                  int bestValue) const;
    int lmrReduction(const Move& move, int i, int depthLeft, int ply, bool inCheck) const;
    int searchMove(Board& board, const Move& move, int i, int movesSearched, int alpha,
                   int beta, int depthLeft, int ply, bool inCheck);
    void onQuietCutoff(Board& board, const Move& move, int depthLeft, int ply,
                       const Move* quietsTried, int quietsTriedCount);
    void updatePV(int ply, const Move& move);
    int correctedEval(const Board& board, int rawEval);
    void updateCorrection(const Board& board, int depthLeft, int bestValue, int staticEval, int flag);

    static constexpr int CORR_SIZE = 16384;
    static constexpr int CORR_GRAIN = 256;
    static constexpr int CORR_MAX = 32 * CORR_GRAIN;
    static constexpr int CORR_WEIGHT_SCALE = 512;
    static constexpr int CORR_MIN_DEPTH = 3;
    std::vector<int> pawnCorr = std::vector<int>(2 * CORR_SIZE, 0);

    int scoreCapture(Board& board, Move& move, bool* losing = nullptr);
    int scoreMove(Board& board, Move& move, int depth);
    void updateHistory(const Board& board, const Move& move, int ply, int bonus);

    // Search stack: the move made at each ply and the piece that made it
    // (piece -1 = no usable move, e.g. a null move).
    Move stackMoves[64];
    int stackPieces[64];

    // Countermove: the quiet reply that last refuted [piece][to] of the previous move.
    Move counterMoves[12][64];

    // Continuation history: [previous piece][previous to][piece][to].
    // Heap-allocated (about 2.4 MB) because Search lives on the stack in main().
    std::vector<int> contHist = std::vector<int>(12 * 64 * 12 * 64, 0);
    int& contHistEntry(int prevPiece, int prevTo, int piece, int to) {
        return contHist[((prevPiece * 64 + prevTo) * 12 + piece) * 64 + to];
    }

    static constexpr int SCORE_COUNTER      =   700000;

    // Move-ordering score bands (see scoreMove).
    static constexpr int SCORE_HASH         =  2000000;
    static constexpr int SCORE_GOOD_CAPTURE =  1000000;
    static constexpr int SCORE_KILLER_1     =   900000;
    static constexpr int SCORE_KILLER_2     =   800000;
    static constexpr int SCORE_BAD_CAPTURE  = -1000000;
    static constexpr int SCORE_UNDERPROMO   = -2000000;
    static constexpr int HISTORY_MAX        =    10000;
    static constexpr int QS_SKIP            = -10000000;  // quiescence: losing capture

    // Mate scores: being mated at ply p scores -MATE_SCORE + p. Anything beyond
    // +/-MATE_BOUND is a mate score (static evals never get near it).
    static constexpr int MATE_SCORE = 90000;
    static constexpr int MATE_BOUND = 80000;

    // The TT stores mate scores relative to the node, the search relative to the root.
    static int scoreToTT(int score, int ply) {
        return score > MATE_BOUND ? score + ply : score < -MATE_BOUND ? score - ply : score;
    }
    static int scoreFromTT(int score, int ply) {
        return score > MATE_BOUND ? score - ply : score < -MATE_BOUND ? score + ply : score;
    }

    // Reverse futility pruning: return the static eval at shallow non-PV nodes
    // when it beats beta by RFP_MARGIN per remaining ply.
    static constexpr int RFP_MAX_DEPTH = 7;
    static constexpr int RFP_MARGIN = 80;

    // Late move pruning: at non-PV nodes with depth <= LMP_MAX_DEPTH, search at most
    // LMP_BASE + depth^2 legal quiet moves and skip the rest.
    static constexpr int LMP_MAX_DEPTH = 4;
    static constexpr int LMP_BASE = 3;

    // SEE pruning at non-PV nodes with depth <= SEE_PRUNE_MAX_DEPTH: skip quiets with
    // SEE < -SEE_QUIET_MARGIN * depth^2 and captures with SEE < -SEE_CAPTURE_MARGIN * depth.
    static constexpr int SEE_PRUNE_MAX_DEPTH = 6;
    static constexpr int SEE_QUIET_MARGIN = 50;
    static constexpr int SEE_CAPTURE_MARGIN = 100;

    // Quiescence generates quiet checking moves only in its first QS_CHECK_PLIES
    // plies (0 = never); check evasions are always searched.
    static constexpr int QS_CHECK_PLIES = 0;
    // UCI score string: "cp N", or "mate N" in moves (negative = getting mated).
    static std::string scoreToUci(int score);

    uint64_t betaCutoffs = 0;
    uint64_t firstMoveCutoffs = 0;
    void pickBestMove(Move* moveList, int count, int currentIndex);
    

    GenerateMoves gen;

    TimeManagement timeManager;

    Move bestMoveFound;
    bool timed = false;
    bool rootDepthDone = false;
    int historyTable[2][64][64] = {};   // kept between searches, so it must start at zero

    Move killerMoves[64][2];

    const int MAX_PLY = 64;

    uint64_t nodes;
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;

    // Triangular PV table: pvTable[ply][ply .. pvLength[ply]-1] is the best line
    // found from the node at that ply. One extra row because a child at ply 64
    // still resets its length before returning. Kept last so this large, rarely
    // touched block does not sit between the hot move-ordering tables.
    int pvLength[65];
    Move pvTable[65][65];
};