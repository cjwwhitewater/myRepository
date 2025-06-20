#include <iostream>
#include <cmath>
#include <cassert>
#include "options.h"
#include "DroneStatusQuerier.h"
#include "dji_fc_subscription.h"

using namespace std;

DroneStatusQuerier::DroneStatusQuerier()
{
    T_DjiReturnCode code = DjiFcSubscription_SubscribeTopic(
                                     DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                     DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to subscribe to get drone height"
             << endl;
        return;
    }

    code = DjiFcSubscription_SubscribeTopic(
                                     DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                     DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to subscribe to get drone velocity"
             << endl;
        return;
    }

    code = DjiFcSubscription_SubscribeTopic(
                                     DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                     DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to subscribe to get drone quaternion"
             << endl;
        return;
    }

    // Absolute position.
    code = DjiFcSubscription_SubscribeTopic(
                                     DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                     DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to subscribe to get fused position"
             << endl;
        return;
    }
}

DroneStatusQuerier::~DroneStatusQuerier()
{
    T_DjiReturnCode code;
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to unsubscribe topic"
                " DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION"
             << endl;
        return;
    }

    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to unsubscribe topic"
                " DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY"
             << endl;
        return;
    }

    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to unsubscribe topic"
                " DJI_FC_SUBSCRIPTION_TOPIC_GIMBAL_ANGLES"
             << endl;
        return;
    }

    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to unsubscribe topic"
                " DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION"
             << endl;
    }
}


float DroneStatusQuerier::getForwardingSpeed()
{
    Eigen::Vector3d v = getVelocityToBody();
    return v[0];
}

Eigen::Vector3d DroneStatusQuerier::getVelocityToBody()
{
    // Get the velocity relative to NED.
    Eigen::Vector3d velocityToNED;
    getVelocityToNED(velocityToNED);

    // Get the drone orientation.
    float yaw, pitch, roll;
    getDroneOrientation(yaw, pitch, roll);

    return convertVelocityNEDToBodyFRD(velocityToNED, yaw, pitch, roll);
}

// This function is generated by deepseek.
Eigen::Vector3d DroneStatusQuerier::convertVelocityNEDToBodyFRD(
                                    const Eigen::Vector3d& velocityToNED,
                                    float yaw, float pitch, float roll)
{
    // 将欧拉角从度转换为弧度（假设输入是度数）
    // 如果输入已经是弧度，可以去掉这部分
    yaw   = yaw   * M_PI / 180.0f;
    pitch = pitch * M_PI / 180.0f;
    roll  = roll  * M_PI / 180.0f;

    // 计算旋转矩阵的元素
    // 偏航(yaw)旋转 - 绕Z轴
    const double cosYaw = cos(yaw);
    const double sinYaw = sin(yaw);

    // 俯仰(pitch)旋转 - 绕Y轴
    const double cosPitch = cos(pitch);
    const double sinPitch = sin(pitch);

    // 横滚(roll)旋转 - 绕X轴
    const double cosRoll = cos(roll);
    const double sinRoll = sin(roll);

    // 构建从NED到FRD的旋转矩阵
    // 这是三个旋转矩阵的组合：R = R_roll * R_pitch * R_yaw
    Eigen::Matrix3d R;

    R(0,0) = cosPitch * cosYaw;
    R(0,1) = cosPitch * sinYaw;
    R(0,2) = -sinPitch;

    R(1,0) = sinRoll * sinPitch * cosYaw - cosRoll * sinYaw;
    R(1,1) = sinRoll * sinPitch * sinYaw + cosRoll * cosYaw;
    R(1,2) = sinRoll * cosPitch;

    R(2,0) = cosRoll * sinPitch * cosYaw + sinRoll * sinYaw;
    R(2,1) = cosRoll * sinPitch * sinYaw - sinRoll * cosYaw;
    R(2,2) = cosRoll * cosPitch;

    // 应用旋转矩阵将速度从NED转换到FRD
    Eigen::Vector3d velocityToBodyFRD = R * velocityToNED;

    return velocityToBodyFRD;
}

void DroneStatusQuerier::getVelocityToNED(Eigen::Vector3d& v)
{
    v.fill(0);

    T_DjiFcSubscriptionVelocity djiVelocity;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                     DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                     (uint8_t*)&djiVelocity,
                     sizeof(djiVelocity),
                     NULL);

    if (djiVelocity.health == DJI_FC_SUBSCRIPTION_DATA_NOT_HEALTH)
        return;

    v[0] = djiVelocity.data.x;
    v[1] = djiVelocity.data.y;
    // The velocity returned from PSDK to relative to NEU.
    // To obtain a velocity relative to NED, the sign of the z-axis should
    // be reverted.
    v[2] = - djiVelocity.data.z;
}

void DroneStatusQuerier::getDroneOrientation(float& yaw, float& pitch, float&roll)
{
    T_DjiReturnCode returnCode;
    T_DjiFcSubscriptionQuaternion quaternion;
    returnCode = DjiFcSubscription_GetLatestValueOfTopic(
                     DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                     (uint8_t*)&quaternion,
                     sizeof(quaternion),
                     NULL);

    // convert quaternion to yaw/pitch/roll.
    double w = quaternion.q0;
    double x = quaternion.q1;
    double y = quaternion.q2;
    double z = quaternion.q3;

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // covnert from radis to degree
    yaw   = yaw   * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    roll  = roll  * 180 / M_PI;
    //cout << "drone yaw=" << yaw << ", pitch=" << pitch
    //     << ", roll="<< roll << endl;
}

float DroneStatusQuerier::getDroneYaw()
{
    float yaw, pitch, roll;
    getDroneOrientation(yaw, pitch, roll);
    return yaw;
}

DronePosition DroneStatusQuerier::getDronePosition()
{
    // get position of drone.
    T_DjiFcSubscriptionPositionFused positionFused;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                (uint8_t*)&positionFused,
                sizeof(positionFused),
                NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to get fused position" << endl;
        return DronePosition();
    }

    DronePosition position;
    position.longitude = positionFused.longitude;
    position.latitude  = positionFused.latitude;

    // get postion of the home position.
    T_DjiFcSubscriptionAltitudeOfHomePoint homeAltitude;
    code = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
        (uint8_t*)&homeAltitude,
        sizeof(homeAltitude),
        NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to get home position" << endl;
        return DronePosition();
    }

    position.height = positionFused.altitude - homeAltitude;

    return position;
}

void DroneStatusQuerier::reportStatus(Json::Value& status)
{
    DronePosition position = getDronePosition();
    status["droneLongitude"] = position.longitude / M_PI * 180;
    status["droneLatitude"]  = position.latitude  / M_PI * 180;
    status["droneHeight"]    = position.height;

    float yaw, pitch, roll;
    getDroneOrientation(yaw, pitch, roll);
    status["droneYaw"]   = yaw;
    status["dronePitch"] = pitch;

    Eigen::Vector3d v = getVelocityToBody();
    status["droneVelocityX"]   = v[0];
    status["droneVelocityY"]   = v[1];
    status["droneVelocityZ"]   = v[2];
}

// Runtime singletn.
DroneStatusQuerier* DroneStatusQuerier::singleton = nullptr;
DroneStatusQuerier* DroneStatusQuerier::createInstance()
{
    assert(singleton == nullptr);
    singleton = new DroneStatusQuerier();
    return singleton;
}

DroneStatusQuerier* DroneStatusQuerier::get()
{
    assert(singleton);
    return singleton;
}
