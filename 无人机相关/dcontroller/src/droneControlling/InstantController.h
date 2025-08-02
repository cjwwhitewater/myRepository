#pragma once
#include "JoystickCommand.h"

class InstantController{
public:
    // Determine a joystick command to be sent to the drone for the current
    // time point. Each derived class should override this virtual function
    // to implement its own determination logic.
    virtual JoystickCommand determineCurrentCommand() = 0;
};
