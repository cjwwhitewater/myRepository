#include <iostream>
#include <algorithm>
#include <cassert>
#include "options.h"
#include "Attacker.h"
#include <jetgpio.h>
#include "nanotimer.h"

using namespace std;

Attacker::Attacker()
{
    // Initialize the GPIO port.
    gpioSetMode(pinNumber, JET_OUTPUT);
}

Attacker::~Attacker()
{
    gpioWrite(pinNumber, 0);  // turn off the laser emitter when exit.
}

void Attacker::singleShot()
{
    gpioWrite(pinNumber, 1);
    millisecond_delay(1000);
    gpioWrite(pinNumber, 0);
}

void Attacker::multipleShots()
{
    options::Options& ops = options::OptionsInstance::get();
    int shotTimes = ops.getInt("shotTimesInMultipleShotsMode", 5);
    for (int i=0; i<shotTimes; i++){
        gpioWrite(pinNumber, 1);
        millisecond_delay(1000);
        gpioWrite(pinNumber, 0);
        millisecond_delay(500);
    }
}

// runtime singleton
Attacker* Attacker::singleton = nullptr;

Attacker* Attacker::createInstance()
{
    assert(singleton == nullptr);
    singleton = new Attacker();
    return singleton;
}

Attacker* Attacker::get()
{
    assert(singleton);
    return singleton;
}
