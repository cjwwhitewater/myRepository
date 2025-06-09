#pragma once
#include <string>

// To coorperate with the Waypoint Mission of PSDK, we define this class.,
// It uses the (absolute) longtitude and latitude to represent the horizental
// position of the drone, but use the relative height to the home-position
// to describe its vertical position.
class DronePosition {
public:
    DronePosition();

    // Return true if all the members are zeros.
    bool isInvalid();

    std::string toString();

    double longitude;
    double latitude;
    // relative to the home-position. Positve number represent an altitude above
    // the home-position.
    double height;
};

