#include <cassert>
#include <cmath>
#include <iostream>
#include <fstream>
#include "options.h"
#include "SafetyChecker.h"
#include "../droneControlling/ControlModeManager.h"

using namespace std;

SafetyChecker::SafetyChecker()
{
    instructions.insert( {D3, D4, D5, D6, D9, D10, D22} );
}

SafetyChecker::~SafetyChecker()
{
}

void SafetyChecker::setETTPackager(shared_ptr<Packager> packager_)
{
    packager = packager_;
}

bool SafetyChecker::checkInstruction(Instruction instruction, double speed)
{
    if (instructions.count(instruction) == 0 ){
        // All other instructions are always regarded as valid.        
        return true;
    }

    bool isSafe = false;
    double obstacleDistance = 0;
    switch (instruction){
      case D3: // forward
        obstacleDistance = perceptor.getForwardObstacleDistance();
        break;
      case D4: // backward
        obstacleDistance = perceptor.getBackwardObstacleDistance();
        break;
      case D5: // left
        obstacleDistance = perceptor.getLeftObstacleDistance();
        break;
      case D6: // right
        obstacleDistance = perceptor.getRightObstacleDistance();
        break;
      case D22: // down
        obstacleDistance = perceptor.getDownObstacleDistance();
        break;

      case D9: // speed
        obstacleDistance = perceptor.getForwardObstacleDistance();
        break;

      case D10: // accelerating
        obstacleDistance = perceptor.getForwardObstacleDistance();
        break;
    }

    double brakingDistance = determineBrakingDistance(instruction, speed);
    isSafe = brakingDistance < obstacleDistance;

    options::Options& ops = options::OptionsInstance::get();
    if (ops.presents("displaySafetyCheckingDetail")){
        static ofstream os("tem.txt");
        os << "check safety: "
           << "brakingDistance = "   << brakingDistance << ", "
           << "obstacle distance = " << obstacleDistance << ", "
           << "isSafe = " << isSafe << endl;
    }

    return isSafe;
}

double SafetyChecker::determineBrakingDistance(Instruction instruction,
                                               double speed)
{
    options::Options& ops = options::OptionsInstance::get();
    static double maximumSpeed = ops.getDouble("maximumSpeed", 5);
    static double brakingDistanceAtMaximumSpeed =
            ops.getDouble("brakingDistanceAtMaximumSpeed", 20.0);

    double minimumSafeDistance;
    switch (instruction){
      case D3:
      case D4:
      case D5:
      case D6:
      case D9:
      case D10:
        minimumSafeDistance =
                    ops.getDouble("minimumHorizontalSafeDistance", 5);
        break;
      case D22:
        minimumSafeDistance =
                    ops.getDouble("minimumDownwardSafeDistance", 2);
        break;
    }

    double brakingDistance = speed / maximumSpeed * brakingDistanceAtMaximumSpeed;
    // Make the drone far away enough to the surrounding objects.
    double d = std::max(brakingDistance, minimumSafeDistance);

    return d;
}

void SafetyChecker::sendCheckingResult(int essentialTextID,
                                       Instruction instruction,
                                       bool isValid)
{
    Json::Value reply;
    reply["checkedEssentialTextID"]  = essentialTextID;
    reply["essentialTextType"] = "instructionCheckResult";
    reply["sourceDevice"]      = "drone";
    reply["instructionID"]     = (string)magic_enum::enum_name(instruction);
    if ( isValid ) {
        reply["checkResult"] = "valid";
    }else{
        reply["checkResult"] = "invalid";
    }

    packager->writeText( reply.toStyledString());
}

void SafetyChecker::sendEmptyCheckingResult(int essentialTextID,
                                            Instruction instruction)
{
    Json::Value reply;
    reply["checkedEssentialTextID"]  = essentialTextID;
    reply["essentialTextType"] = "instructionCheckResult";
    reply["sourceDevice"]      = "drone";
    reply["instructionID"]     = (string)magic_enum::enum_name(instruction);
    // No 'checkResult' member.

    packager->writeText( reply.toStyledString());
}

// runtime singleton
SafetyChecker* SafetyChecker::singleton = nullptr;

SafetyChecker* SafetyChecker::createInstance()
{
    assert(singleton == nullptr);
    singleton = new SafetyChecker();
    return singleton;
}

SafetyChecker* SafetyChecker::get()
{
    assert(singleton);
    return singleton;
}
