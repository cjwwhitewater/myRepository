#pragma once
#include <string>
#include <cmath>
#include <queue>
#include <jsoncpp/json/json.h>
#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_typedef.h"
#include "FlighModule.h"
#include "Instruction.h"
#include "PersistentCommandExecuter.h"
#include "InstantCommandExecuter.h"
#include "InstantController.h"
#include "InstructionFollowingController.h"
#include "AccelerationController.h"
#include "RTHController.h"
#include "WaypointMissionController.h"

using std::string;

enum ControlMode {
    OnGroundMode,
    HoveringMode,
    InstructionFollowingMode,
    AcceleratingMode,
    ReturnToHomeMode,
    WaypointMissionMode,
    UnspecifiedMode
};

class ControlModeManager {
public:
    // Let the FlighModule to initialize the flight control module of PSDK,
    // and let it to setup safe-paramters of drone.
    void initialize();

    // Determine whether the specified instruction can be processed by this class.
    bool canProcessInstruction(const string& instructionName);

    // Process the specified instruction.
    // This is the kernel function of this class.
    void processInstruction(const Json::Value& instruction);

    // When a timer event reaches, this function is executed.
    // It performs different actions according to the current control mode.
    // For example, if in InstructionFollowingMode, the current instant controller
    // will be informed to determine the current joystick command to be executed.
    // Then this command is sent to drone for execution.
    void executeTimerTask();

    ControlMode getControlMode() const;

    // Report this class's status.
    void reportStatus(Json::Value& status);

private:
    void switchControlMode(ControlMode newMode);

    // Apply the control over joystick.
    bool applyJoystickControl();

private:
    ControlMode controlMode;

    std::map<string, Instruction> instructions;

private:
    FlightModule flightModule;
    PersistentCommandExecuter persistentCommandSender;

    // This is a upcasted pointer, it is set to point to one of the
    // following controllers.
    InstantController* instantController;

    // A factory of controllers. We create these object beforehand to
    // avoid frequent construction and destruction on the heap.
    InstructionFollowingController instructionFollowingController;
    AccelerationController accelerationController;
    RTHController rthController;
    WaypointMissionController waypointMissionController;

// runtime singleton
public:
    static ControlModeManager* createInstance();
    static ControlModeManager* get();
private:
    ControlModeManager();
    ~ControlModeManager();

private:
    static ControlModeManager* singleton;
};
