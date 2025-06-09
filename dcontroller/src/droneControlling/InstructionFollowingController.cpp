#include <iostream>
#include <cassert>
#include "InstructionFollowingController.h"
#include "DroneStatusQuerier.h"

using namespace std;

InstructionFollowingController::InstructionFollowingController()
{
}


void InstructionFollowingController::setInstruction(
                                            Instruction instruction_,
                                            double instructionParameter)
{
    instruction = instruction_;

    // reset the counter to be zero.
    valueChangeCounter = 0;

    switch (instruction){
      case D3:  // forward
        targetValue     = 0.5;
        // We let the execution the speed changing of the
        // foward flighing spend 3 seconds.
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;
      case D4:  // backward
        targetValue = -0.5;
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;
      case D5:  // turn-left
        targetValue = -0.5;
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;
      case D6:  // turn-right
        targetValue = 0.5;
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;
      case D21: // ascending
        targetValue = 0.5;
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;
      case D22: // descending
        targetValue = -0.5;
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;
      case D9:  // speed control
        targetValue = instructionParameter;
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;

      case D11:  // pitch control
        targetValue = instructionParameter;
        valueChangeStep = 0;  // not used at all.
        break;

      case D13:{  // yaw control
        double yawToBody = instructionParameter;
        targetValue = determineExpectedYawToGround(yawToBody);
        valueChangeStep = 2;  // in degree
        break;
      }
    case D24:
        targetValue = 0.5;
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;
    case D25:
        targetValue = 0.5;
        valueChangeStep = targetValue / (ComandFrequency * 3);
        break;

      default:
        cout << "Error: InstructionFollowingController doesn't support "
                "instruction " << instruction << endl;
    }
}

double InstructionFollowingController::determineExpectedYawToGround(
                                                        double yawToBody)
{
    // Determine the expected raw relative to ground.
    double yawToGround = DroneStatusQuerier::get()->getDroneYaw();
    double expectedYawToGround = yawToGround + yawToBody;
    if (expectedYawToGround > 180 )
        expectedYawToGround -= 360;
    else if (expectedYawToGround < -180 )
        expectedYawToGround += 360;

    return expectedYawToGround;
}

JoystickCommand InstructionFollowingController::determineCurrentCommand()
{
    switch (instruction){
      case D3:
        return determineCommandForForwarding();
      case D4:
        return determineCommandForBackwarding();
      case D5:
        return determineCommandForTurningLeft();
      case D6:
        return determineCommandForTurningRight();
      case D21:
        return determineCommandForAscending();
      case D22:
        return determineCommandForDescending();
      case D9:
        return determineCommandForSpeedControl();
      case D11:
        return determineCommandForPitchControl();
      case D13:
        return determineCommandForYawControl();
      case D24:
        return determineCommandForLeftForwardControl();
      case D25:
        return determineCommandForRightForwardControl();

      default:
        cout << "Error: InstructionFollowingController doesn't support "
                "instruction " << instruction << endl;
    }
    return JoystickCommand();
}

double InstructionFollowingController::determineCurrentValue()
{
    double currentValue;
    currentValue = valueChangeCounter++ * valueChangeStep;
    if ( fabs(currentValue) > fabs(targetValue) ) {
        currentValue = targetValue;
        valueChangeCounter--;  // to prevent counter overflow.
    }
    return currentValue;
}

JoystickCommand InstructionFollowingController::determineCommandForForwarding()
{
    JoystickCommand command;
    command.command.x   = determineCurrentValue();
    command.command.y   = 0;    // no velocity component along y-axis.
    command.command.z   = 0;    // no velocity component along z-axis.
    command.command.yaw = 0;    // no change for the yaw; so keep the current yaw.

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForBackwarding()
{
    JoystickCommand command;
    command.command.x   = determineCurrentValue();
    command.command.y   = 0;      // no velocity component along y-axis.
    command.command.z   = 0;      // no velocity component along z-axis.
    command.command.yaw = 0;      // no change for the yaw; so keep the
                                  // current yaw.

    return command;
}


JoystickCommand InstructionFollowingController::determineCommandForTurningLeft()
{
    JoystickCommand command;
    command.command.x   = 0;      // no velocity component along x-axis.
    command.command.y   = determineCurrentValue();
    command.command.z   = 0;      // no velocity component along z-axis.
    command.command.yaw = 0;      // no change for the yaw; so keep the
                                  // current yaw.

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForTurningRight()
{
    JoystickCommand command;
    command.command.x   = 0;      // no velocity component along x-axis.
    command.command.y   = determineCurrentValue();
    command.command.z   = 0;      // no velocity component along z-axis.
    command.command.yaw = 0;      // no change for the yaw; so keep the
                                  // current yaw.

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForAscending()
{
    JoystickCommand command;
    command.command.x   = 0;      // no velocity component along x-axis.
    command.command.y   = 0;      // no velocity component along y-axis.
    command.command.z   = determineCurrentValue();
    command.command.yaw = 0;      // no change for the yaw; so keep the
                                  // current yaw.

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForDescending()
{
    JoystickCommand command;
    command.command.x   = 0;      // no velocity component along x-axis.
    command.command.y   = 0;      // no velocity component along y-axis.
    command.command.z   = determineCurrentValue();
    command.command.yaw = 0;      // no change for the yaw; so keep the
                                  // current yaw.

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForSpeedControl()
{
    JoystickCommand command;
    command.command.x   = determineCurrentValue();
    command.command.y   = 0;     // no velocity component along y-axis.
    command.command.z   = 0;     // no velocity component along z-axis.
    command.command.yaw = 0;     // no change for the yaw; so keep the current yaw.

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForPitchControl()
{
    JoystickCommand command;

    // This control mode can control the pitch and roll of drone.
    command.mode.horizontalControlMode =
            DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE;
    // It is easier to use the ground frame to
    // control of the pitch of drone.
    command.mode.horizontalCoordinate =
            DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE;

    static map<double, double> mapper = {
        // column 1: expected pitch
        // column 2: adjusted real pitch
        // column 3: expected pitch
        // column 4: observed/real pitch
        {-11, -12 - 0.1},     //{-11, -12},
        {-10, -12},           //{-10, -13},
        {-9,  -12 + 0.1},     //{-9,  -12},
        {-8,  -9},            //{-8,  -9},
        {-7,  -8 - 0.1},      //{-7,  -8},
        {-6,  -8},            //{-6,  -9},
        {-5,  -7},            //{-5,  -7},
        {-4,  -6},            //{-4,  -6},
        {-3,  -3},            //{-3,  -3},
        {-2,  -2},            //{-2,  -2},
        {-1,  -1},            //{-1,  -1},
        {0,    1.5},          //{0,    3},
        {1,    2},            //{1,    2},
        {2,    3.5},          //{2,    3.5},
        {3,    5.5},          //{3,    5.5},
        {4,    9  },          //{4,    9  },
        {5,    11 - 0.2 },    //{5,    11 },
        {6,    11 - 0.1},     //{6,    11 },
        {7,    11 },          //{7,    11 },
        {8,    12 - 0.2},     //{8,    12 },
        {9,    12 },          //{9,    12 },
        {10,   13 },          //{10,   13 },
        {11,   15 },          //{11,   15 },
        {12,   17 },          //{12,   17 },
        {13,   18 },          //{13,   18 },
        {14,   19 },          //{14,   19 },
        {15,   20 },          //{15,   20 },
        {16,   22 - 0.2},     //{16,   22 },
        {17,   22 },          //{17,   22 },
        {18,   23 - 0.2},     //{18,   23 },
        {19,   23 },          //{19,   23 },
        {20,   24.5 - 0.1 },  //{20,   24.5},
        {21,   24.5 - 0.09},  //{21,   24.5},
        {22,   24.5 - 0.08},  //{22,   24.5},
        {23,   24.5 - 0.07},  //{23,   24.5},
        {24,   24.5 - 0.06},  //{24,   24.5},
        {25,   24.5 - 0.05},  //{25,   24.5},
        {26,   24.5 - 0.04},  //{26,   24.5},
        {27,   24.5 - 0.03},  //{27,   24.5},
        {28,   24.5 - 0.02},  //{28,   24.5},
        {29,   24.5 - 0.01},  //{29,   24.5},
        {30,   24.5}          //{30,   24.5}
    };
    double nearestExpectedPitch = targetValue;
    double minDiff = 1000;
    for (auto it=mapper.begin(); it != mapper.end(); it++){
        double diff = fabs(it->second - targetValue);
        if ( diff < minDiff ){
            minDiff = diff;
            nearestExpectedPitch = it->first;
        }
    }
    command.command.x   = 0;                      // roll
    command.command.y   = nearestExpectedPitch;   // pitch
    command.command.z   = 0;     // no velocity component along z-axis.
    command.command.yaw = 0;     // no change for the yaw;
                                 // so keep the current yaw.

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForYawControl()
{
    // determine the joystick command.
    JoystickCommand command;
    // Make the x/y components of the command represent the pitch/roll.
    command.mode.horizontalControlMode =
            DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE;
    // For yaw control, we should set the following control mode for the
    // yaw section.
    command.mode.yawControlMode =
            DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE;
    // The frame must be ground frame.
    command.mode.horizontalCoordinate =
            DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE;

    command.command.x   = 0;     // reset pitch to zero.
    command.command.y   = 0;     // reset roll to zero.
    command.command.z   = 0;     // no velocity component along z-axis.
    command.command.yaw = targetValue;

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForLeftForwardControl()
{
    // determine the joystick command.
    JoystickCommand command;
    double speed = determineCurrentValue();
    command.command.x   =  speed * cos(10.0/180* M_PI);
    command.command.y   = -speed * sin(10.0/180* M_PI);
    command.command.z   = 0;      // no velocity component along z-axis.
    command.command.yaw = 0;      // no change for the yaw; so keep the
                                  // current yaw.

    return command;
}

JoystickCommand InstructionFollowingController::determineCommandForRightForwardControl()
{
    // determine the joystick command.
    JoystickCommand command;
    double speed = determineCurrentValue();
    command.command.x   = speed * cos(10.0/180* M_PI);
    command.command.y   = speed * sin(10.0/180* M_PI);
    command.command.z   = 0;      // no velocity component along z-axis.
    command.command.yaw = 0;      // no change for the yaw; so keep the
                                  // current yaw.

    return command;
}



