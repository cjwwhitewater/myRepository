#include <iostream>
#include <algorithm>
#include <cassert>
#include "nanotimer.h"
#include "options.h"
#include "FanController.h"
#include <jetgpio.h>
#include <unistd.h>

using namespace std;

FanController::FanController()
{
    // Initialize the GPIO port.
    gpioSetMode(pinNumber, JET_OUTPUT);
    gpioSetPWMfrequency(pinNumber, 500);   // in Hz
}

FanController::~FanController()
{
    gpioPWM(pinNumber, 0);   // shutdown the motor when exit.
}

void FanController::setGear(int gear)
{
    std::clamp(gear, 0, 4);
    // The following mapping from the gear to the duty is earier for
    // human visualization than a linear mapping one.
    int mapper[5]={0, 24, 30, 40, 255};
    int duty = mapper[gear];
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
