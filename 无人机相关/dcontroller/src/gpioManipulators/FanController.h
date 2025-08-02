#pragma once
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>


class FanController{
public:
    void setGear(int gear);

private:    
    const int pinNumber = 33;   // physical pin ID.

    // runtime singleton
public:
    static FanController* createInstance();
    static FanController* get();
private:
    FanController();
    ~FanController();

private:
    static FanController* singleton;
};
