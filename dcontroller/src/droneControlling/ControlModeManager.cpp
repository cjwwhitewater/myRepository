#include <cassert>
#include <iostream>
#include "options.h"
#include "magic_enum.hpp"
#include "ControlModeManager.h"
#include "../safetyChecking/SafetyChecker.h"

using namespace std;

ControlModeManager::ControlModeManager()
{
    instructions["D1"] = D1;
    instructions["D2"] = D2;
    instructions["D3"] = D3;
    instructions["D4"] = D4;
    instructions["D5"] = D5;
    instructions["D6"] = D6;
    instructions["D7"] = D7;
    instructions["D8"] = D8;
    instructions["D9"] = D9;
    instructions["D10"] = D10;
    instructions["D11"] = D11;
    instructions["D13"] = D13;
    instructions["D21"] = D21;
    instructions["D22"] = D22;
    instructions["D23"] = D23;
    instructions["D24"] = D24;
    instructions["D25"] = D25;
    instructions["D26"] = D26;

    instructions["D100"] = D100;
    instructions["D101"] = D101;

    controlMode = OnGroundMode;
    instantController  = nullptr;
}

ControlModeManager::~ControlModeManager()
{
    if ( ! flightModule.cleanup()) {
        cout << "failed to cleanup the flight module of PSDK" << endl;
    }
}

void ControlModeManager::initialize()
{
    if ( ! flightModule.initialize()) {
        cout << "failed to initialize the flight module of PSDK" << endl;
        exit(-1);
    }
    if ( ! flightModule.setAndCheckDroneSafeParameters()) {
        cout << "failed to set and check drone safe-parameter" << endl;
        exit(-1);
    }
}

bool ControlModeManager::applyJoystickControl()
{
    T_DjiReturnCode code =
            DjiFlightController_ObtainJoystickCtrlAuthority();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Obtain joystick authority failed, error code: "
             << code << endl;
        return false;
    }

    // Wait the above action to finish.
    millisecond_delay(1000);

    return true;
}

bool ControlModeManager::canProcessInstruction(const string& instructionName)
{
    return instructions.count(instructionName) > 0;
}

void ControlModeManager::processInstruction(const Json::Value &instructionData)
{
    int essentialTextID = instructionData["essentialTextID"].asInt();
    string instructionName  = instructionData["instructionID"].asString();
    if ( instructions.count(instructionName) == 0){
        cout << "ControlModeManager can not process instruction "
             << instructionName << endl;
        return;
    }

    // Apply the joystick control, see doc for the underlying logic.
    if ( !applyJoystickControl()){
        cout << "failed to apply joystick control" << endl;
        return;
    }

    bool isSafe = false;
    Instruction instruction = instructions[instructionName];    
    switch (instruction) {
      case D1: // takeOff
        if (controlMode != OnGroundMode){
            cout << "to takeoff, drone must be in OnGroundMode "
                 << endl;
            return;
        }
        if ( ! persistentCommandSender.takeOff() ){
            cout << "failed to execute the takeoff command" << endl;
            return;
        }        
        SafetyChecker::get()->sendEmptyCheckingResult(essentialTextID, D1);
        switchControlMode(HoveringMode);
        break;

      case D2: // land
        if (controlMode != HoveringMode){
          cout << "to land, drone must be in HoveringMode" << endl;
          return;
        }
        if ( ! persistentCommandSender.land() ){
          cout << "failed to execute the landing command" << endl;
          return;
        }
        SafetyChecker::get()->sendEmptyCheckingResult(essentialTextID, D2);
        switchControlMode(OnGroundMode);
        break;

      //  ==== instructions that can be implemented by
      //  ==== InstructionFollowingController.
      case D3: // forward
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to move forward, drone must be in "
                  "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }
        isSafe = SafetyChecker::get()->checkInstruction(D3, 0.5);
        SafetyChecker::get()->sendCheckingResult(essentialTextID, D3, isSafe);
        if (isSafe){
            instructionFollowingController.setInstruction(
                        D3, 0.5);
            switchControlMode(InstructionFollowingMode);
        }
        break;

      case D4: // backward
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to move backward, drone must be in "
                  "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }
        isSafe = SafetyChecker::get()->checkInstruction(D4, 0.5);
        SafetyChecker::get()->sendCheckingResult(essentialTextID, D4, isSafe);
        if (isSafe){
            instructionFollowingController.setInstruction(
                        D4, 0.5);
            switchControlMode(InstructionFollowingMode);
        }
        break;

      case D5: // turn-left
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to turn left, drone must be in "
                  "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }
        isSafe = SafetyChecker::get()->checkInstruction(D5, 0.5);
        SafetyChecker::get()->sendCheckingResult(essentialTextID,
                                                 D5, isSafe);
        if (isSafe){
            instructionFollowingController.setInstruction(
                        D5, 0.5);
            switchControlMode(InstructionFollowingMode);
        }
        break;

      case D6: // turn-right
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to turn right, drone must be in "
                    "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }

        isSafe = SafetyChecker::get()->checkInstruction(D6, 0.5);
        SafetyChecker::get()->sendCheckingResult(essentialTextID, D6, isSafe);
        if (isSafe){
            instructionFollowingController.setInstruction(
                        D6, 0.5);
            switchControlMode(InstructionFollowingMode);
        }
        break;

      case D21: // ascend
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to ascend, drone must be in "
                    "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }
        SafetyChecker::get()->sendEmptyCheckingResult(essentialTextID, D21);
        instructionFollowingController.setInstruction(
                    D21, 0.5);
        switchControlMode(InstructionFollowingMode);
        break;

      case D22: // descend
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to descend, drone must be in "
                    "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }

        isSafe = SafetyChecker::get()->checkInstruction(D22, 0.5);
        SafetyChecker::get()->sendCheckingResult(essentialTextID, D22, isSafe);
        if (isSafe){
            instructionFollowingController.setInstruction(
                        D22, 0.5);
            switchControlMode(InstructionFollowingMode);
        }
        break;

      case D9:{  // speed
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "for speed control, drone must be in "
                    "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }

        double speed = instructionData["instructionParameter"].asDouble();
        isSafe = SafetyChecker::get()->checkInstruction(D9, speed);
        SafetyChecker::get()->sendCheckingResult(essentialTextID, D9, isSafe);
        if (isSafe){
            instructionFollowingController.setInstruction(
                        D9, speed);
            switchControlMode(InstructionFollowingMode);
        }
        break;
      }

      case D11:{  // change drone pitch
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "for drone pitch control, drone must be in "
                    "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }
        double pitch = instructionData["instructionParameter"].asDouble();

        instructionFollowingController.setInstruction(
                    D11, pitch);
        SafetyChecker::get()->sendEmptyCheckingResult(essentialTextID,
                                                      D11);
        switchControlMode(InstructionFollowingMode);
        break;
      }

      case D13:{  // change yaw
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to change yaw, drone must be in "
                    "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }
        double yaw = 0;
        if (instruction == D13){
            yaw = instructionData["instructionParameter"].asInt();
            // yaw range for the instruction is [0,360], we convert this
            // value to a yaw-representation in the range of [-180,180].
            if (yaw > 180)
                yaw -= 360;
        }else if (instruction == D24)
            yaw = -10;  // in degree
        else
            yaw =  10;
        instructionFollowingController.setInstruction(instruction,
                                                                yaw);
        SafetyChecker::get()->sendEmptyCheckingResult(essentialTextID,
                                                      instruction);
        switchControlMode(InstructionFollowingMode);
        break;
      }
      case D24:
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to fly with yaw=-10 degree, drone must be in "
                  "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }
        isSafe = SafetyChecker::get()->checkInstruction(D24, 0.5);
        SafetyChecker::get()->sendCheckingResult(essentialTextID,
                                                 D24, isSafe);
        if (isSafe){
            instructionFollowingController.setInstruction(
                        D24, 0.5);
            switchControlMode(InstructionFollowingMode);
        }
        break;

      case D25:
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ) ) {
            cout << "to fly with yaw=10 degree, drone must be in "
                    "HoveringMode or InstructionFollowingMode" << endl;
            return;
        }
        isSafe = SafetyChecker::get()->checkInstruction(D25, 0.5);
        SafetyChecker::get()->sendCheckingResult(essentialTextID,
                                                 D25, isSafe);
        if (isSafe){
            instructionFollowingController.setInstruction(
                        D25, 0.5);
            switchControlMode(InstructionFollowingMode);
        }
        break;

      // ==== the hover instruction
      case D7:  // hover
        if (! ( controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ||
                controlMode == AcceleratingMode ||
                controlMode == WaypointMissionMode) ){
          cout << "to hover, drone must be in InstructionFollowingMode or"
                  " AcceleratingMode" << endl;
          return;
        }
        SafetyChecker::get()->sendEmptyCheckingResult(essentialTextID, D7);
        switchControlMode(HoveringMode);
        break;

      // ==== acceleration instruction
      case D10:{  // accelerate
        if ( ! (controlMode == HoveringMode ||
                controlMode == AcceleratingMode ) ) {
            cout << "for acceleration control, drone must be in "
                    "HoveringMode or AcceleratingMode" << endl;
            return;
        }
        isSafe = SafetyChecker::get()->checkInstruction(
                               D10,
                               accelerationController.getMaximumSpeed() );
        SafetyChecker::get()->sendCheckingResult(essentialTextID,
                                                 D10, isSafe);
        if ( isSafe ){
            double targetAcc =
                    instructionData["instructionParameter"].asDouble();
            accelerationController.startControlling(targetAcc);
            switchControlMode(AcceleratingMode);
        }
        break;
      }

      // ==== Waypoint management instructions
      case D100:      
        if ( ! (controlMode == HoveringMode) ) {
            cout << "to manage waypoints, drone must be in HoveringMode"
                 << endl;
            return;
        }
        waypointMissionController.addWaypoint();
        break;

      case D101:
        waypointMissionController.deleteWaypoint();
        break;

        // ==== Waypoint management instructions
      case D8:
      case D23:
        if ( ! (controlMode == HoveringMode) ) {
            cout << "to perform waypoint mission, drone must be in HoveringMode"
                 << endl;
            return;
        }
        SafetyChecker::get()->sendEmptyCheckingResult(essentialTextID,
                                                      instruction);
        if (instruction == D8 ){
            if ( waypointMissionController.performWaypointMission())
                switchControlMode(WaypointMissionMode);
            return;
        }
        if (instruction == D23 ){
            if (waypointMissionController.gotoTargetPosition())
                switchControlMode(WaypointMissionMode);
            return;
        }
        break;

      // ==== Return-To-Home instruction
      case D26:{  // Return-To-Home
        if ( ! (controlMode == HoveringMode ||
                controlMode == InstructionFollowingMode ||
                controlMode == AcceleratingMode ) ) {
            cout << "to return-to-home, drone must be in "
                    "HoveringMode, InstructionFollowingMode or AcceleratingMode"
                 << endl;
            return;
        }
        SafetyChecker::get()->sendEmptyCheckingResult(essentialTextID, D26);
        rthController.startReturnToHome();
        switchControlMode(ReturnToHomeMode);
        break;
      }
   } // swtich
}

void ControlModeManager::executeTimerTask()
{
    if (  controlMode == InstructionFollowingMode ||
          controlMode == AcceleratingMode
       ) {
        JoystickCommand command = instantController->determineCurrentCommand();
        InstantCommandExecuter::get()->setCommand(command);
        return;
    }

    // If in Waypoint Mission mode, and the drone has finished the mission,
    // switch to HoveringMode.
    if ( controlMode == WaypointMissionMode){
        if (waypointMissionController.isMissionTerminated()){
            switchControlMode(HoveringMode);
            cout << "drone has finished the waypoint mission, switched to "
                    "hovering mode" << endl;
        }
        return;
    }

    // If in Return-To-Home mode, and the drone has just grounded,
    // switch to the OnGroundMode.
    if ( controlMode == ReturnToHomeMode){
        if ( rthController.isDroneGrounded()){            
            switchControlMode(OnGroundMode);
            cout << "drone has grounded, so put it in OnGroundMode" << endl;
        }
        return;
    }
}

void ControlModeManager::switchControlMode(ControlMode newMode)
{
    // For some old modes, some cleanup action should be
    // performed.
    if (controlMode == WaypointMissionMode){
        waypointMissionController.cancelMission();
    }

    // Peform some action according to the new mode.
    controlMode = newMode;
    switch (controlMode) {
    case InstructionFollowingMode:
        instantController = & instructionFollowingController;
        break;
    case AcceleratingMode:
        instantController = & accelerationController;
        break;    
    case UnspecifiedMode:
        break;
    }

    cout << "ControlModeManager switched to "
         << magic_enum::enum_name(controlMode) << endl;
}

ControlMode ControlModeManager::getControlMode() const
{
    return controlMode;
}

void ControlModeManager::reportStatus(Json::Value& status)
{
    status["ControlMode"] = (string)magic_enum::enum_name(controlMode);
    status["waypoints"] = waypointMissionController.waypointsToString();
    if (controlMode == AcceleratingMode ){
        status["averageAcceleration"] =
                accelerationController.getRecentAccelerationAverage();
    }
}

// Runtime singletn.
ControlModeManager* ControlModeManager::singleton = nullptr;
ControlModeManager* ControlModeManager::createInstance()
{
    assert(singleton == nullptr);
    singleton = new ControlModeManager();
    return singleton;
}

ControlModeManager* ControlModeManager::get()
{
    assert(singleton);
    return singleton;
}
