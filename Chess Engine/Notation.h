#pragma once
#include <string>
#include "Types.h"

// Text forms of squares and moves (UCI long algebraic: e2e4, e7e8q).
namespace Notation {
    // "e4" for square 28; "??" outside the board.
    std::string square(int sq);
    // 'q', 'r', 'b' or 'n' for a promotion, '\0' otherwise.
    char promotionChar(const Move& move);
    // UCI string of a move, e.g. "e2e4" or "e7e8q".
    std::string moveToUci(const Move& move);
}
