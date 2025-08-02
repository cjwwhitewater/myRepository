#pragma once

class FlightModule{
public:
    // Initialize the flight control module of PSDK.
    // Return true if succeeded.
    bool initialize();

    // Set and check some parameters of the drone to ensure that the drone
    // is in safe condition for taking off.
    // Return true if succeeded.
    bool setAndCheckDroneSafeParameters();

    // clean up the flight control module of PSDK.
    // Return true if succeeded.
    bool cleanup();
};
