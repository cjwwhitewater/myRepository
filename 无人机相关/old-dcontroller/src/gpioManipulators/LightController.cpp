#include <iostream>
#include <algorithm>
#include <cassert>
#include "options.h"
#include "LightController.h"
#include <jetgpio.h>
#include <unistd.h>

using namespace std;

LightController::LightController()
{
    // Initialize the GPIO port.
    gpioSetMode(pinNumber, JET_OUTPUT);    
    gpioSetPWMfrequency(pinNumber, 100);   // in Hz
}

LightController::~LightController()
{
    gpioPWM(pinNumber, 0);   // turn off the light when exit.
}

void LightController::setBrightness(int brightness)
{
    std::clamp(brightness, 0, 4);
    int duty = brightness * 255 / 4;
    gpioPWM(pinNumber, duty);
}

// runtime singleton
LightController* LightController::singleton = nullptr;

LightController* LightController::createInstance()
{
    assert(singleton == nullptr);
    singleton = new LightController();
    return singleton;
}

LightController* LightController::get()
{
    assert(singleton);
    return singleton;
}
