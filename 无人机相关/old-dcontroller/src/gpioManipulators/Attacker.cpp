#include <iostream>
#include <algorithm>
#include <cassert>
#include "options.h"
#include "Attacker.h"
#include <jetgpio.h>
#include <unistd.h>

using namespace std;

Attacker::Attacker()
{
    // Initialize the GPIO port.
    gpioSetMode(pinNumber, JET_OUTPUT);
}

Attacker::~Attacker()
{
    gpioPWM(pinNumber, 0);   // shutdown the motor when exit.
}

void Attacker::singleShot(int duration_ms)
{
    gpioWrite(pinNumber, 1);
    usleep(duration_ms * 1000); // convert to microseconds
    // cjwnote: 这里的usleep函数是一个阻塞函数，可能会导致其他线程无法执行。
    gpioWrite(pinNumber, 0);
}

void Attacker::multipleShots(int times, int duration_ms, int interval_ms)
{
    for (int i=0; i<times; i++){
        gpioWrite(pinNumber, 1);
        usleep(duration_ms * 1000);
        gpioWrite(pinNumber, 0);
        if(i!=times-1){
            usleep(interval_ms * 1000);
        }
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
