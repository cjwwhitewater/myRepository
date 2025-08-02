#include <sstream>
#include "JoystickCommand.h"

using namespace std;

JoystickCommand::JoystickCommand()
{
    // Set the control mode for each section, those modes will server as
    // the default setting.

    // horizental : velocity
    mode.horizontalControlMode =
            DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE;

    // vertical : velocity
    mode.verticalControlMode =
            DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE;

    // yaw: angle
    mode.yawControlMode =
            DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE;

    // coordinate: body
    mode.horizontalCoordinate  =
            DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE;

    // stable control: enabled.
    mode.stableControlMode     =
            DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE;

    command.x   = 0;
    command.y   = 0;
    command.z   = 0;
    command.yaw = 0;
}

string JoystickCommand::toString()
{
    ostringstream os;
    os << "{ mode:{ horizental = ";
    switch (mode.horizontalControlMode){
      case DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE:
           os << "angle"; break;
      case DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE:
           os << "velocity"; break;
      case DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE:
           os << "position"; break;
      case DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGULAR_RATE_CONTROL_MODE:
           os << "angular_rate"; break;
    }
    os << ", ";

    os << "vertical = ";
    switch (mode.verticalControlMode){
      case DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE:
           os << "velocity"; break;
      case DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE:
           os << "position"; break;
      case DJI_FLIGHT_CONTROLLER_VERTICAL_THRUST_CONTROL_MODE:
           os << "thrust"; break;
    }
    os << ", ";

    os << "yaw = ";
    switch (mode.yawControlMode){
      case DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE:
           os << "angle"; break;
      case DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE:
           os << "angle_rate"; break;
    }
    os << ", ";

    os << "coordinate = ";
    switch (mode.horizontalCoordinate){
      case DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE:
           os << "ground"; break;
      case DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE:
           os << "body"; break;
    }
    os << ", ";

    os << "stable = ";
    switch (mode.stableControlMode){
      case DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_DISABLE:
           os << "disabled"; break;
      case DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE:
           os << "enabled"; break;
    }
    os << "}, ";

    os << "{ command:{ "
       << "x = " << command.x << ", "
       << "y = " << command.y << ", "
       << "z = " << command.z << ", "
       << "yaw = " << command.yaw;
    os << "} }";

    return os.str();
}
