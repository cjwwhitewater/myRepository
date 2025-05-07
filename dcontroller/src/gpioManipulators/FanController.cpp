#include <iostream>
#include <algorithm>
#include <cassert>
#include "options.h"
#include "FanController.h"
#include <jetgpio.h>
#include <unistd.h>

using namespace std;

FanController::FanController()
{
    // Initialize the GPIO port.
    gpioSetMode(pinNumber, JET_OUTPUT);
    // 额外常量
    gpioSetPWMfrequency(pinNumber, 50);   // in Hz
}

FanController::~FanController()
{
    gpioPWM(pinNumber, 0);   // shutdown the motor when exit.
}

void FanController::setGear(int gear)
{
    std::clamp(gear, 0, 4);
    int duty = gear * 255 / 4;
    gpioPWM(pinNumber, duty);
}

// runtime singleton
FanController* FanController::singleton = nullptr;

FanController* FanController::createInstance()
{
    assert(singleton == nullptr);
    singleton = new FanController();
    return singleton;
}

FanController* FanController::get()
{
    assert(singleton);
    return singleton;
}
