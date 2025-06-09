#include <iostream>
#include <sstream>
#include "options.h"
#include "WaypointMissionController.h"
#include "DroneStatusQuerier.h"


using namespace std;

// Since the protocol of the callback function doesn't allow passing
// the this pointer, we need to use the following static data and functions.
static bool waypointMissionTerminated = 0;

WaypointMissionController::WaypointMissionController()
{
    T_DjiReturnCode code = DjiWaypointV2_Init();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to initialize PSDK WaypointV2 module" << endl;
        return;
    }

    // To ensure that the mission information is available within
    // the entire time period of the waypoint mission, we construct
    // this data structure in this constructor, and destruct it
    // in the class destructor.
    mission.mission = new T_DjiWaypointV2[WayPointMaxNumber];
}

WaypointMissionController::~WaypointMissionController()
{
    delete[] mission.mission;

    T_DjiReturnCode code = DjiWaypointV2_Deinit();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to de-initialize PSDK WaypointV2 module" << endl;
        return;
    }
}

void WaypointMissionController::addWaypoint()
{
    options::Options& ops = options::OptionsInstance::get();

    DronePosition currentPosition =
            DroneStatusQuerier::get()->getDronePosition();
    if ( currentPosition.isInvalid() &&
         ops.presents("WaypointMustBeValidGPSPosition") ){
        cout << "current position is invalid, no waypoint added"  << endl;
        return;
    }
    waypoints.push_back(currentPosition);
    while (waypoints.size() > WayPointMaxNumber )
        waypoints.pop_front();

    cout << waypointsToString() << endl;
}

void WaypointMissionController::deleteWaypoint()
{
    if (waypoints.empty()) return;
    waypoints.pop_back();

    cout << waypointsToString() << endl;
}

bool WaypointMissionController::performWaypointMission()
{
    if (waypoints.size() != 3){
        cout << "to perform a waypoint mission, "
                "Extactly three waypoints should have been set"
             << endl;
        return false;
    }

    flyThroughWaypoints(waypoints);
    return true;
}

bool WaypointMissionController::gotoTargetPosition()
{
    if (waypoints.size() != 1){
        cout << "to perform a waypoint mission, "
                "Extactly one waypoint should have been set"
             << endl;
        return false;
    }

    deque<DronePosition> points;
    // Append the real target position
    points.push_front( waypoints.front() );

    // Append norminal waypoints.
    DronePosition pos = waypoints.front();
    pos.height += 2;
    points.push_front( pos);
    pos.height += 2;
    points.push_front( pos);

    flyThroughWaypoints(points);
    return true;
}

void WaypointMissionController::cancelMission()
{
    T_DjiReturnCode code = DjiWaypointV2_Stop();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to stop the waypoint mission" << endl;
        waypointMissionTerminated = 1;
        return;
    }
}

static T_DjiReturnCode wayointMissionStateCallback(
                       T_DjiWaypointV2MissionStatePush stateData)
{
    if (stateData.state == DJI_WAYPOINT_V2_MISSION_STATE_EXIT_MISSION)
        waypointMissionTerminated = 1;

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void WaypointMissionController::flyThroughWaypoints(
                                     const deque<DronePosition>& positions)
{
    waypointMissionTerminated = 0;

    // construct the mission description.
    // Basic settings.
    static int missionID = 0;
    mission.missionID    = missionID++;
    mission.repeatTimes  = 0;
    // When the mission finish, we don't let the drone to perform
    // any other action such as go-home.
    mission.finishedAction  = DJI_WAYPOINT_V2_FINISHED_NO_ACTION;
    mission.maxFlightSpeed  = 5;    // in meter/s
    mission.autoFlightSpeed = 5;    // in meter/s
    // If RC signal lose, don't continue the mission.
    mission.actionWhenRcLost =
            DJI_WAYPOINT_V2_MISSION_STOP_WAYPOINT_V2_AND_EXECUTE_RC_LOST_ACTION;
    mission.gotoFirstWaypointMode =
            DJI_WAYPOINT_V2_MISSION_GO_TO_FIRST_WAYPOINT_MODE_SAFELY;
    // We don't want perform any waypoint action.
    mission.actionList.actionNum = 0;
    mission.actionList.actions   = nullptr;

    // waypoints description.
    mission.missTotalLen = positions.size();
    for (int i=0; i<positions.size(); i++){
        T_DjiWaypointV2& wp = mission.mission[i];
        // most important settings.
        wp.longitude        = positions[i].longitude;
        wp.latitude         = positions[i].latitude;
        wp.relativeHeight   = positions[i].height;

        // other settings.
        wp.waypointType =
                DJI_WAYPOINT_V2_FLIGHT_PATH_MODE_GO_TO_POINT_ALONG_CURVE;
        wp.headingMode  = DJI_WAYPOINT_V2_HEADING_MODE_AUTO;
        wp.config.useLocalCruiseVel = 0;
        wp.config.useLocalMaxVel    = 0;
        // When the drone is approaching a waypoint and the distance is
        // reduced to the value below, the drone begin to adjust its speed.
        wp.dampingDistance = 5;     // in meter.
        wp.maxFlightSpeed  = 5;     // in meter/s
        wp.autoFlightSpeed = 5;     // in meter/s
    }

    // upload the mission information to the drone.
    T_DjiReturnCode code = DjiWaypointV2_UploadMission(&mission);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to upload waypoint mission" << endl;
        waypointMissionTerminated = 1;
        return;
    }

    // register a callback function to receive the state information when
    // performing the mission.
    code = DjiWaypointV2_RegisterMissionStateCallback(wayointMissionStateCallback);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to register the callback function to "
                "receive state of waypoint mission"
             << endl;
        waypointMissionTerminated = 1;
        return;
    }

    options::Options& ops = options::OptionsInstance::get();
    if (ops.presents("dryRun")){
        cout << "the mission has been uploaded to drone." << endl
             << "dryRun: a waypoint mission" << endl;
        waypointMissionTerminated = 1;
        return;
    }

    // Initiate the waypoint mission.
    code = DjiWaypointV2_Start();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to start the waypoint mission" << endl;
        waypointMissionTerminated = 1;
        return;
    }
}

bool WaypointMissionController::isMissionTerminated()
{
    return waypointMissionTerminated;
}

string WaypointMissionController::waypointsToString()
{
    std::ostringstream os;        
    for (int i=0; i<waypoints.size(); i++){
        os << "WP" << i << ": "
           << waypoints[i].toString() << " ";
    }
    return os.str();
}
