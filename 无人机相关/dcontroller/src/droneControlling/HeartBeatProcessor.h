#pragma once
#include "nanotimer.h"

class HeartBeatProcessor {
public:
    void updateHeartBeatTimeStamp();
    bool lastHeartBeatIsFresh();

private:
    nanotimer timer;
    double lastHeartBeatTimeStamp;

// runtime singleton
public:
    static HeartBeatProcessor* createInstance();
    static HeartBeatProcessor* get();
private:
    HeartBeatProcessor();
    ~HeartBeatProcessor();

private:
    static HeartBeatProcessor* singleton;
};
