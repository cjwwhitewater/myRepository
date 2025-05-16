#include <iostream>
#include "options.h"
#include "GimbalDriver.h"
#include "nanotimer.h"
#include "dji_fc_subscription.h"
#include "dji_gimbal_manager.h"

using namespace std;

GimbalDriver* GimbalDriver::singleton = nullptr;

GimbalDriver::GimbalDriver()
{
    mountPosition = DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1;
}

GimbalDriver::~GimbalDriver()
{
    T_DjiReturnCode returnCode;
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to unsubscribe topic DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES"
             << endl;
        return;
    }

    returnCode = DjiGimbalManager_Deinit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to deinit gimbal manager" << endl;
        return;
    }
}

void GimbalDriver::initialize()
{
    // Subscript to get gimbal pose.
//    T_DjiAircraftInfoBaseInfo baseInfo;
//    T_DjiReturnCode returnCode = DjiAircraftInfo_GetBaseInfo(&baseInfo);
//    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
//        cout << "In GimbalDriver::initialize(), failed to get aircraft base info"
//             << endl;
//        return;
//    }
//    E_DjiAircraftSeries aircraftSeries = baseInfo.aircraftSeries;
//    E_DjiMountPosition mountPosition =

    T_DjiReturnCode returnCode = DjiFcSubscription_SubscribeTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "In GimbalDriver::initialize(), failed to subscribe to get gimbal pose"
             << endl;
        return;
    }

    returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "In GimbalDriver::initialize(), failed to nit gimbal manager" << endl;
        return;
    }

    returnCode = DjiGimbalManager_SetMode(mountPosition, DJI_GIMBAL_MODE_FREE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "In GimbalDriver::initialize(), failed to set gimbal mode" << endl;
        return;
    }

    returnCode = DjiGimbalManager_Reset(mountPosition,
                                        DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "In GimbalDriver::initialize(), failed to reset gimbal" << endl;
        return;
    }
}

void GimbalDriver::setYaw(int yawInDegree)
{
    float currentYaw, currentPitch, currentRoll;
    getGimbalPose(currentYaw, currentPitch, currentRoll);

    T_DjiReturnCode returnCode;
    T_DjiGimbalManagerRotation rotation;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
    rotation.yaw   = yawInDegree;
    rotation.pitch = currentPitch;
    rotation.roll  = currentRoll;
    returnCode = DjiGimbalManager_Rotate(mountPosition, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed for DjiGimbalManager_Rotate()" << endl;
        return;
    }
}

void GimbalDriver::setPitch(int pitchInDegree)
{
    float currentYaw, currentPitch, currentRoll;
    getGimbalPose(currentYaw, currentPitch, currentRoll);

    T_DjiReturnCode returnCode;
    T_DjiGimbalManagerRotation rotation;
    rotation.rotationMode = DJI_GIMBAL_ROTATION_MODE_ABSOLUTE_ANGLE;
    rotation.yaw   = currentYaw;
    rotation.pitch = pitchInDegree;
    rotation.roll  = currentRoll;
    returnCode = DjiGimbalManager_Rotate(mountPosition, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed for DjiGimbalManager_Rotate()" << endl;
        return;
    }
}

void GimbalDriver::getGimbalPose(float& yaw, float& pitch)
{
    float roll;
    getGimbalPose(yaw, pitch, roll);
}

void GimbalDriver::getGimbalPose(float& yaw, float& pitch, float&roll)
{
    T_DjiReturnCode returnCode;
    T_DjiFcSubscriptionGimbalAngles gimbalAngles = {0};
    returnCode = DjiFcSubscription_GetLatestValueOfTopic(
                        DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES,
                        (uint8_t*)& gimbalAngles,
                        sizeof(T_DjiFcSubscriptionGimbalAngles),
                        NULL);
    // With experiments, meaning of the x/y/z fields are as follows; it doesn't
    // obey the FRD(forward-right-down) frame. I think it might be a bug of PSDK.
    pitch = gimbalAngles.x;
    roll  = gimbalAngles.y;
    yaw   = gimbalAngles.z;
    //cout << "current yaw = " << yaw << ", "
    //     << "pitch = " << pitch << ", roll = " << roll
    //     << endl;
}

// runtime singleton
void GimbalDriver::createInstance()
{
    if (singleton){
        cout << "the singleton of GimbalDriver has already been created, "
                "don't try to create it again" << endl;
        exit(-1);
    }
    singleton = new GimbalDriver;
    singleton->initialize();
}


GimbalDriver* GimbalDriver::get()
{
    if (singleton==nullptr){
        cout << "GimbalDriver::get() being called but the singleton of "
                "GimbalDriver has not been created" << endl;
        exit(-1);
    }
    return singleton;
}
