#include "TimeManagement.h"
#include <algorithm>


void TimeManagement::startTimer(int timeLeft, int increment, int movesToGo, int phase) {
    startTime = std::chrono::high_resolution_clock::now();
    if(movesToGo <= 0){
        movesToGo = 35;
    }

    int baseTimeMs = static_cast<int>((timeLeft / movesToGo) + (increment * 0.6));

    double timeMultiplier = 1.3 - (0.5 * phase / 256.0);

    allocatedTimeMs = static_cast<int>(baseTimeMs * timeMultiplier);

    // An iteration that starts late is usually cut off and wasted, so stop starting
    // new ones well before the budget is used; one that is already running may go
    // past the budget up to the hard limit.
    softTimeMs = allocatedTimeMs * 6 / 10;
    hardTimeMs = std::min(allocatedTimeMs * 3, timeLeft / 3);
    if (hardTimeMs >= timeLeft - 50) hardTimeMs = timeLeft - 50;
    if (hardTimeMs < 10) hardTimeMs = 10;
    if (softTimeMs > hardTimeMs) softTimeMs = hardTimeMs;
}

void TimeManagement::startFixed(int moveTimeMs) {
    startTime = std::chrono::high_resolution_clock::now();
    // Small safety margin for output and GUI overhead.
    allocatedTimeMs = moveTimeMs > 40 ? moveTimeMs - 20 : moveTimeMs / 2 + 1;
    softTimeMs = hardTimeMs = allocatedTimeMs;
}

bool TimeManagement::isTimeUp() {
    return getElapsedTimeMs() >= hardTimeMs;
}

bool TimeManagement::isSoftTimeUp() {
    return getElapsedTimeMs() >= softTimeMs;
}

int TimeManagement::getElapsedTimeMs() {
    auto now = std::chrono::high_resolution_clock::now();
    return static_cast<int> (std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count());
}
