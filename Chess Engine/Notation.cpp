#include "Notation.h"

namespace Notation {

std::string square(int sq) {
    if (sq < 0 || sq > 63) return "??";
    return { (char)('a' + sq % 8), (char)('1' + sq / 8) };
}

char promotionChar(const Move& move) {
    switch (move.flag) {
        case MoveFlag::PROMOTION_QUEEN:
        case MoveFlag::CPROMOTION_QUEEN:  return 'q';
        case MoveFlag::PROMOTION_ROOK:
        case MoveFlag::CPROMOTION_ROOK:   return 'r';
        case MoveFlag::PROMOTION_BISHOP:
        case MoveFlag::CPROMOTION_BISHOP: return 'b';
        case MoveFlag::PROMOTION_KNIGHT:
        case MoveFlag::CPROMOTION_KNIGHT: return 'n';
        default:                          return '\0';
    }
}

std::string moveToUci(const Move& move) {
    std::string s = square(move.from) + square(move.to);
    if (move.isPromotion()) s += promotionChar(move);
    return s;
}

}  // namespace Notation
