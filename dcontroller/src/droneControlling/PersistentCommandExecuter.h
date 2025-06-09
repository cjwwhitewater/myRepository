#pragma once

class PersistentCommandExecuter{
public:
    PersistentCommandExecuter();

    // Return true if the command is executed successfully.
    bool takeOff();

    // Return true if the command is executed successfully.
    bool land();

 private:
    // Check whether the motor has been started.
    bool isMotorStarted();

    // Check whether the drone has taken off and thus is in-air.
    bool isDroneInAir();

    // Check whether the drone has finished the taking-off command.
    bool isTakingOffFinished();

    // Check whether the auto-landing command has been started.
    bool isAutoLandingStarted();
};
