#include <iostream>
#include "cameraControlling.h"
#include "GimbalDriver.h"

using namespace std;

void processPTZControlInstruction(const Json::Value& instruction)
{
    string id = instruction["instructionID"].asString();
    cout << "processing instruction "<< id << endl;

    if ( id == "D16"){
        int focalLength = instruction["instructionParameter"].asInt();
    }
    else if ( id == "D17"){
        int pitch = instruction["instructionParameter"].asInt();
        GimbalDriver::get()->setPitch(pitch);
    }
    else if ( id == "D18"){
        int yaw = instruction["instructionParameter"].asInt();
        GimbalDriver::get()->setYaw(yaw);
    }
    cout << id << " processed" << endl;
}

