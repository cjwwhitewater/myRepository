#include <iostream>
#include "Perceptor.h"
#include "dji_fc_subscription.h"
#include "options.h"

using namespace std;

Perceptor::Perceptor()
{
    T_DjiReturnCode code = DjiFcSubscription_SubscribeTopic(
                                     DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
                                     DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to subscribe to get obstacle information"
             << endl;
        return;
    }
}

Perceptor::~Perceptor()
{
    T_DjiReturnCode code;
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Failed to unsubscribe topic"
                " DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA"
             << endl;
        return;
    }
}

double Perceptor::getForwardObstacleDistance()
{
    T_DjiFcSubscriptionAvoidData obstacleInfo;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                     DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
                     (uint8_t*)&obstacleInfo,
                     sizeof(obstacleInfo),
                     NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        cout << "failed to get obstacle information" << endl;
        return false;
    }
    if (obstacleInfo.frontHealth)
        return obstacleInfo.front;

    // If no obstacle information available, return zero to inform
    // the client to stop flight.
    return 0;
}

double Perceptor::getBackwardObstacleDistance()
{
    T_DjiFcSubscriptionAvoidData obstacleInfo;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                     DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
                     (uint8_t*)&obstacleInfo,
                     sizeof(obstacleInfo),
                     NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        cout << "failed to get obstacle information" << endl;
        return false;
    }
    if (obstacleInfo.backHealth)
        return obstacleInfo.back;

    // If no obstacle information available, return zero to inform
    // the client to stop flight.
    return 0;
}

double Perceptor::getLeftObstacleDistance()
{
    T_DjiFcSubscriptionAvoidData obstacleInfo;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                     DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
                     (uint8_t*)&obstacleInfo,
                     sizeof(obstacleInfo),
                     NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        cout << "failed to get obstacle information" << endl;
        return false;
    }
    if (obstacleInfo.leftHealth)
        return obstacleInfo.left;

    // If no obstacle information available, return zero to inform
    // the client to stop flight.
    return 0;
}

double Perceptor::getRightObstacleDistance()
{
    T_DjiFcSubscriptionAvoidData obstacleInfo;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                     DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
                     (uint8_t*)&obstacleInfo,
                     sizeof(obstacleInfo),
                     NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        cout << "failed to get obstacle information" << endl;
        return false;
    }
    if (obstacleInfo.rightHealth)
        return obstacleInfo.right;

    // If no obstacle information available, return zero to inform
    // the client to stop flight.
    return 0;
}

double Perceptor::getDownObstacleDistance()
{
    T_DjiFcSubscriptionAvoidData obstacleInfo;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                     DJI_FC_SUBSCRIPTION_TOPIC_AVOID_DATA,
                     (uint8_t*)&obstacleInfo,
                     sizeof(obstacleInfo),
                     NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        cout << "failed to get obstacle information" << endl;
        return false;
    }
    if (obstacleInfo.downHealth)
        return obstacleInfo.down;

    // If no obstacle information available,
    options::Options& ops = options::OptionsInstance::get();
    double norminalDownwardObstacleDistance =
            ops.getDouble("norminalDownwardObstacleDistance", 5.0);
    return norminalDownwardObstacleDistance;
}
