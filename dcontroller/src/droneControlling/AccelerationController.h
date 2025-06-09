#pragma once

#include <deque>
#include <jsoncpp/json/json.h>
#include "nanotimer.h"
#include "InstantController.h"

class AccelerationController: public InstantController{
public:
    AccelerationController();

    void startControlling(double targetAcceleration);

    JoystickCommand determineCurrentCommand() override;

    // This function returns the average of the recent accelerations, so that
    // the user can compare it with the target acceleration, to observe the
    // performance of this controller.
    double getRecentAccelerationAverage();

    // Return the maximum (forwarding) speed when in this mode.
    double getMaximumSpeed();

private:
    // Each time this function is called, the expected speed is determined.
    void determineTargetSpeed(double& speed);

private:
    double targetAcceleration;
    double lastSpeed;            // last speed calculated.
    double currentAcceleration;  // last acceleration calculated.
    nanotimer timer;

    // PID controlling variables.
    double KpAcc, KiAcc;
    double errorAccumulated;

    // Period between two adjacent acceleration measures is about 20ms,
    // so the following length corresponds to a duration of one second.
    const int WindowLengthForRecentAccelerations = 20;
    std::deque<double> recentAccelerations;

    // upper limit of the forwarding speed, in meter/s.
    const float MaximumForwardSpeed = 5;
};
