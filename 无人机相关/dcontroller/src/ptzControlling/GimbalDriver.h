#pragma once
#include <string>
#include <utility>
#include <vector>
#include "dji_typedef.h"

using std::string;
using std::vector;
using std::pair;

// As to the reference frames, default one is the ground refernce frame.
// If a quantity is relative to the body frame, we let its name indicating this.
class GimbalDriver{
public:
    ~GimbalDriver();

    // The target yaw is in degree, relative to the drone frame.
    void setGimbalYaw(int targetYawInDroneFrame);

    // The target yaw is in degree, relative to the ground frame.
    void setGimbalPitch(int pitch);

    // Returned angles are in degree and relative to the ground frame.
    void  getGimbalYawAndRoll(float& yaw, float& roll);
    float getGimbalYaw();
    float getGimbalPitch();

    // Report gimal yaw(to body frame) and pitch(to ground frame)
    void reportStatus(Json::Value& status);

private:
    void initialize();

    // Returned angles are in degree and relative to the ground frame.
    void getGimbalPose(float& yaw, float& pitch, float&roll);

    // Return gimbal raw to the body frame.
    float getGimbalYawToBody();

private:
    E_DjiMountPosition mountPosition;

// runtime singleton
public:
    static void createInstance();
    static GimbalDriver* get();
private:
    GimbalDriver();
    GimbalDriver(const GimbalDriver&);
private:
    static GimbalDriver* singleton;
};
