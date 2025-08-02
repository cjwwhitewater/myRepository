#include <iostream>
#include <cmath>
#include "options.h"
#include "nanotimer.h"
#include "PersistentCommandExecuter.h"
#include "nanotimer.h"
#include "dji_fc_subscription.h"
#include "dji_gimbal_manager.h"
#include "dji_flight_controller.h"

using namespace std;

PersistentCommandExecuter::PersistentCommandExecuter()
{
}

bool PersistentCommandExecuter::takeOff()
{
    options::Options& ops = options::OptionsInstance::get();
    if (ops.presents("dryRun")){
        cout << "dryRun: the takeOff command" << endl;
        return true; // return true to pretend that
                     // the drone has been taken off.
    }


    T_DjiReturnCode code = DjiFlightController_StartTakeoff();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Request to take off failed, error code: "
             << code << endl;
        return false;
    }

    //Check whether the motor has been started.
    if (!isMotorStarted()){
        cout << "Takeoff failed. Motors are not spinning." << endl;
        return false;
    }

    // Check whether the drone is in air.
    if (!isDroneInAir()){
        cout << "Takeoff failed. Aircraft is still on the ground, "
                "but the motors are spinning" << endl;
        return false;
    }

    // Check whether the entire takeoff action has been accomplished.
    if (!isTakingOffFinished()){
        cout << "Takeoff finished, but the aircraft is in an "
                "unexpected mode." << endl;
        return false;
    }
    return true;
}

bool PersistentCommandExecuter::land()
{
    options::Options& ops = options::OptionsInstance::get();
    if (ops.presents("dryRun")){
        cout << "dryRun: the land command" << endl;
        return true;
    }

    T_DjiReturnCode code = DjiFlightController_StartLanding();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        cout << "Request to take off failed, error code: "
             << code << endl;
        return false;
    }

    // Check whether the drone enters the AUTO_LANDING mode.
    if (!isAutoLandingStarted()){
        cout << "Fail to execute Landing action!" << endl;
        return false;
    }

    millisecond_delay(5000);
    return true;
}

bool PersistentCommandExecuter::isMotorStarted()
{
    nanotimer timer;
    timer.start();
    while (true) {
        T_DjiFcSubscriptionFlightStatus flightStatus = {0};
        T_DjiFcSubscriptionDisplaymode displayMode = {0};

        T_DjiReturnCode code = DjiFcSubscription_GetLatestValueOfTopic(
                    DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                    (uint8_t *)&flightStatus,
                    sizeof(T_DjiFcSubscriptionFlightStatus),
                    NULL);

        if (code == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            code = DjiFcSubscription_GetLatestValueOfTopic(
                        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                        (uint8_t *)&displayMode,
                        sizeof(T_DjiFcSubscriptionDisplaymode),
                        NULL);
        }

        if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            cout << "Get value of topic flight status error, "
                 << "error code:" << code << endl;
            return false;
        }

        // 检查是否满足启动条件
        if (flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND ||
                displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START) {
            return true;
        }

        if (timer.get_elapsed_ms() > 2000){
            cout << "check motor-starting time out" << endl;
            return false;
        }
    }
}

bool PersistentCommandExecuter::isDroneInAir()
{
    nanotimer timer;
    timer.start();

    while (true) {
        T_DjiFcSubscriptionFlightStatus flightStatus = {0};
        T_DjiFcSubscriptionDisplaymode displayMode = {0};

        T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
                    DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                    (uint8_t *)&flightStatus,
                    sizeof(T_DjiFcSubscriptionFlightStatus),
                    NULL);

        if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            returnCode = DjiFcSubscription_GetLatestValueOfTopic(
                        DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                        (uint8_t *)&displayMode,
                        sizeof(T_DjiFcSubscriptionDisplaymode),
                        NULL);
        }

        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            cout << "Get value of topic flight status error,"
                 << " error code: " << returnCode << endl;
            return false;
        }

        // 检查是否满足离地条件
        if (flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR ||
           (displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
            displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF)) {
            return true;
        }
        if(timer.get_elapsed_ms() >11000){
            cout << "drone in air time out" << endl;
            return false;
        }

        millisecond_delay(50);
    }
}

bool PersistentCommandExecuter::isTakingOffFinished()
{
    while (true) {
        T_DjiFcSubscriptionDisplaymode displayMode = {0};
        T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
                    DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                    (uint8_t *)&displayMode,
                    sizeof(T_DjiFcSubscriptionDisplaymode),
                    NULL);

        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            return false;
        }

        // 如果仍在起飞模式，继续等待
        if (displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF ||
                displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF) {
            millisecond_delay(1000);
            continue;
        }

        // 检查是否进入正常飞行模式
        return (displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
                displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE);
    }
}


bool PersistentCommandExecuter::isAutoLandingStarted()
{
    nanotimer timer;
    timer.start();
    while (true) {
        T_DjiFcSubscriptionDisplaymode displayMode = {0};
        T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
                    DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                    (uint8_t *)&displayMode,
                    sizeof(T_DjiFcSubscriptionDisplaymode),
                    NULL);

        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            return false;
        }

        if (displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING) {
            return true;
        }

        if (timer.get_elapsed_ms() > 2000){
            cout << "auto-landing failed due to time-out" << endl;
            return false;
        }
    }
}

