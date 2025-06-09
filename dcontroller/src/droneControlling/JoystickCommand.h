#pragma once
#include <string>

#include "dji_flight_controller.h"

// Represent a joystick command. It includes both the command paramters themself and
// the control modes for all sections.
class JoystickCommand {
public:
    JoystickCommand();

    // convert the command object into a string-representation for displaying
    // in console. This function is used for debugging.
    std::string toString();

    T_DjiFlightControllerJoystickMode mode;
    T_DjiFlightControllerJoystickCommand command;
};
