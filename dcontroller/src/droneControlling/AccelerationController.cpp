#include <iostream>
#include <fstream>
#include <cmath>
#include <cassert>
#include "options.h"
#include "AccelerationController.h"
#include "DroneStatusQuerier.h"

using namespace std;

AccelerationController::AccelerationController()
{
    options::Options& ops = options::OptionsInstance::get();

    targetAcceleration = 0;

    // set the PID parameters.
    KpAcc = ops.getDouble("KpAcc", 0.02);
    KiAcc = ops.getDouble("KiAcc", 0.01);
    errorAccumulated = 0.0;
}

void AccelerationController::startControlling(double targetAcceleration_)
{
    targetAcceleration = targetAcceleration_;

    // reset PID status.
    errorAccumulated = 0.0;

    // For acceleration measure.
    DroneStatusQuerier* querier = DroneStatusQuerier::get();
    lastSpeed = querier->getForwardingSpeed();
    timer.start();

    recentAccelerations.clear();
}

JoystickCommand AccelerationController::determineCurrentCommand()
{
    JoystickCommand command;

    double speed;
    determineTargetSpeed(speed);

    // The specified value will let the drone don't change its current
    // yaw.
    command.mode.yawControlMode =
            DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE;

    command.command.x   = speed;
    command.command.y   = 0;     // no velocity component along y-axis.
    command.command.z   = 0;     // no velocity component along z-axis.
    command.command.yaw = 0;     // no change for the yaw; so keep the current yaw.

    return command;
}

void AccelerationController::determineTargetSpeed(double& targetSpeed)
{
    double currentSpeed = DroneStatusQuerier::get()->getForwardingSpeed();

    // if current speed has already reached the maximum, don't change it.
    if ( currentSpeed >= MaximumForwardSpeed ){
        targetSpeed = currentSpeed;
        recentAccelerations.clear();
        return;
    }

    // if current speed has already reached zero, set it to be zero and return.
    if ( targetAcceleration < 0 && currentSpeed <= 0.05 ){
        targetSpeed = 0;
        recentAccelerations.clear();
        return;
    }

    // calculate the current acceleration.
    double duration = timer.get_elapsed_ms()/1000.0;
    currentAcceleration = (currentSpeed - lastSpeed)/ duration;

    // record the value of current acceleration.
    recentAccelerations.push_back(currentAcceleration);
    while (recentAccelerations.size() > WindowLengthForRecentAccelerations){
        recentAccelerations.pop_front();
    }

    // determine target speed
    double error = targetAcceleration - currentAcceleration;
    errorAccumulated += error;
    double deltaK = KpAcc * error + KiAcc * errorAccumulated;
    double Delta0 = targetAcceleration * duration;
    targetSpeed = currentSpeed + Delta0 + deltaK;

    // For next measure of acceleration.
    lastSpeed = currentSpeed;
    timer.start();
}

double AccelerationController::getRecentAccelerationAverage()
{
    if (recentAccelerations.empty())
        return 0.0;

    double sum = 0.0;
    for (double v: recentAccelerations){
        sum+=v;
    }
    return sum/recentAccelerations.size();
}

double AccelerationController::getMaximumSpeed()
{
    return MaximumForwardSpeed;
}

