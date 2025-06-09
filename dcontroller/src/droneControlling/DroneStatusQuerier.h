#pragma once
#include <Eigen/Core>
#include "DronePosition.h"
#include <jsoncpp/json/json.h>

class DroneStatusQuerier {
public:
    // == Velocity information ==
    // Return the speed along the x-axis/forwarding direction of the
    // drone-frame.
    float getForwardingSpeed();

    // == Orientation information ==
    // Returned angles are in degree and relative to the ground frame.
    // Ranges of the returned angles:
    // yaw   : -180 ~ 180.  Positive corresponds to turning right.
    // pitch : -90  ~ 90.
    // roll  : -180 ~ 180.
    void getDroneOrientation(float& yaw, float& pitch, float&roll);
    // helper functions of the above.
    float getDroneYaw();

    // == Absolute position information ==
    DronePosition getDronePosition();

    // Report drone's position, orientation and velocity(to body).
    void reportStatus(Json::Value& status);

private:
    // Get the velocity of drone relative to the NED ground frame.
    // Note, there is another 'Vector3d' in PSDK, but that is a vector
    // of integers rather than doubles.
    void getVelocityToNED(Eigen::Vector3d& v);

    // Rotate the input vector by the specified Euler angles, and
    // return the resultant vector.
    Eigen::Vector3d convertVelocityNEDToBodyFRD(
                        const Eigen::Vector3d& velocityToNED,
                        float yaw, float pitch, float roll);

    // Get the velocity of drone relative to the FRD body frame.
    Eigen::Vector3d getVelocityToBody();

// runtime singleton
public:
    static DroneStatusQuerier* createInstance();
    static DroneStatusQuerier* get();
private:
    DroneStatusQuerier();
    ~DroneStatusQuerier();

private:
    static DroneStatusQuerier* singleton;
};


