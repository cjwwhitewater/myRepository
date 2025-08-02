#pragma once
#include "Instruction.h"
#include "InstantController.h"

class InstructionFollowingController: public InstantController {
public:
    InstructionFollowingController();

    // Determine the expected command to execute for the specified instruction.
    void setInstruction(Instruction instruction,
                        double instructionParameter);

    // Return the expected command.
    JoystickCommand determineCurrentCommand() override;

private:    
    // Given the expected yaw relative to the current forwarding direction
    // of the drone, this function determine the corresponding absolute yaw
    // value relative to the ground frame.
    // Range of the input value is 0~360 degree.
    // Range of the result is -180~180 degree.
    double determineExpectedYawToGround(double yawToBody);

    // To make value of the command parameter change gradually, we use
    // a counter to remember how many times the command has been executed.
    // The current value is then determined according to this counter.
    double determineCurrentValue();

    JoystickCommand determineCommandForForwarding();
    JoystickCommand determineCommandForBackwarding();
    JoystickCommand determineCommandForTurningLeft();
    JoystickCommand determineCommandForTurningRight();
    JoystickCommand determineCommandForSpeedControl();
    JoystickCommand determineCommandForPitchControl();
    JoystickCommand determineCommandForYawControl();
    JoystickCommand determineCommandForAscending();
    JoystickCommand determineCommandForDescending();
    JoystickCommand determineCommandForLeftForwardControl();
    JoystickCommand determineCommandForRightForwardControl();

private:
    Instruction instruction;

    // The target value of one instruction.
    double targetValue;

    // Each time one instant command is executed, the command parameter will be
    // increased by the following step, so that the parameter changes gradually
    // to the target one.
    double valueChangeStep;

    // When one instruction is first set, the counter is set to zero.
    // For each execution of the 'determineCurrentCommand', the counter
    // will be increased by one.
    int valueChangeCounter;

    // The command timer has the following execution frequency.
    const int ComandFrequency = 20;
};
