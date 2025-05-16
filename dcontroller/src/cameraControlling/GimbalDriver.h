#pragma once
#include <string>
#include <utility>
#include <vector>
#include "dji_typedef.h"

using std::string;
using std::vector;
using std::pair;

class GimbalDriver{
public:
    ~GimbalDriver();

    void setYaw(int yawInDegree);
    void setPitch(int pitchInDegree);
    // Returned yaw is always relative to the true-north of earch.
    void getGimbalPose(float& yaw, float& pitch);

private:
    void initialize();

    // Returned yaw is always relative to the true-north of earch.
    void getGimbalPose(float& yaw, float& pitch, float&roll);

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
