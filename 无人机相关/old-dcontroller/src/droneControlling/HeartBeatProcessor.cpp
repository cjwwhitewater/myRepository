#include <cassert>
#include <algorithm>
#include "options.h"
#include "HeartBeatProcessor.h"

HeartBeatProcessor::HeartBeatProcessor()
{
    timer.start();
}

void HeartBeatProcessor::updateHeartBeatTimeStamp()
{
    lastHeartBeatTimeStamp = timer.get_elapsed_ms();
}

bool HeartBeatProcessor::lastHeartBeatIsFresh()
{
    double period = timer.get_elapsed_ms() - lastHeartBeatTimeStamp;
    return period < 500;
}

// runtime singleton
HeartBeatProcessor* HeartBeatProcessor::singleton = nullptr;

HeartBeatProcessor* HeartBeatProcessor::createInstance()
{
    assert(singleton == nullptr);
    singleton = new HeartBeatProcessor();
    return singleton;
}

HeartBeatProcessor* HeartBeatProcessor::get()
{
    assert(singleton);
    return singleton;
}
