#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>


class LightController{
public:    
    void setBrightness(int bright);

private:    
    const int pinNumber = 15;   // physical pin ID.

    // runtime singleton
public:
    static LightController* createInstance();
    static LightController* get();
private:
    LightController();
    ~LightController();

private:
    static LightController* singleton;
};
