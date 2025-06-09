#pragma once
#include <string>
#include <deque>
#include "DronePosition.h"
#include "dji_waypoint_v2.h"

using std::string;

class WaypointMissionController{
public:
    WaypointMissionController();
    ~WaypointMissionController();

    // Mark the currente drone position as a new waypoint.
    void addWaypoint();

    // Delete the most recent waypoint(if any).
    void deleteWaypoint();

    // Three waypoints should have been set previously by user. This function
    // make the drone fly through the waypoints.
    // Return true if the mission is to be executed.
    bool performWaypointMission();

    // A single waypoint should have been set previously by user. This function
    // make the drone fly to the target position.
    // Return true if the mission is to be executed.
    bool gotoTargetPosition();

    // For one or three points mission, cancel the mission.
    void cancelMission();

    // Return true if the waypoint mission has been terminated.
    // Reasons to terminate a mission include: failed to start the mission, and
    // the mission has accomplished successfully.
    bool isMissionTerminated();

    // Output the waypoints to a string.
    string waypointsToString();

private:
    // Make the drone fly through all the waypoints.
    void flyThroughWaypoints(
            const std::deque<DronePosition>& positions);

private:
    const int WayPointMaxNumber = 3;
    std::deque<DronePosition> waypoints;

    // A data structure required by PSDK. We should construct this structure
    // beforehand.
    T_DjiWayPointV2MissionSettings mission;
};
