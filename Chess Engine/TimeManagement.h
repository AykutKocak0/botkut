#pragma once
#include <chrono>


class TimeManagement {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> startTime;
    int allocatedTimeMs;   // average budget for this move
    int softTimeMs;        // no new iteration is started after this
    int hardTimeMs;        // the running search is aborted at this point

public:
    void startTimer(int timeLeft, int increment, int movesToGo, int phase);
    // Fixed time per move ("go movetime N").
    void startFixed(int moveTimeMs);
    // Hard limit: checked inside the search.
    bool isTimeUp();
    // Soft limit: checked between iterations.
    bool isSoftTimeUp();
    int getElapsedTimeMs();
    int getAllocatedTimeMs(){return allocatedTimeMs;}
};
