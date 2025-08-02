#include <iostream>
#include "RTHController.h"
#include "options.h"
#include "dji_fc_subscription.h"
#include "dji_flight_controller.h"

using namespace std;

RTHController::RTHController()
{
    T_DjiReturnCode code = DjiFcSubscription_SubscribeTopic(
                                     DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                     DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to subscribe to get drone working mode"
             << endl;
        return;
    }
}

void RTHController::startReturnToHome()
{
    options::Options& ops = options::OptionsInstance::get();
    if (ops.presents("dryRun")){
        cout << "dryRun: Return-To-Home"  << endl;
        return;
    }

    T_DjiReturnCode code = DjiFlightController_StartGoHome();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "failed to initiate the Return-To-Home action"
             << endl;
        return;
    }
}

bool RTHController::isDroneGrounded()
{
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                     DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                     (uint8_t*)&flightStatus,
                     sizeof(flightStatus),
                     NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS){
        cout << "failed to get drone flight status" << endl;
        return false;
    }

    if ( flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_STOPED ||
         flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND )
        return true;

    return false;
}
