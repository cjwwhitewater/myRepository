#include <iostream>
#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_aircraft_info.h"
#include "FlighModule.h"
#include "nanotimer.h"

using namespace std;

bool FlightModule::initialize()
{
    T_DjiFlightControllerRidInfo ridInfo = {0};
    T_DjiReturnCode code = DjiFlightController_Init(ridInfo);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX",
                       code);
        return false;
    }

    code = DjiFcSubscription_Init();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX",
                       code);
        return false;
    }

    code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                            DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                            NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX",
                       code);
        return false;
    }

    // The working mode presented to the user.
    code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic display mode failed, error code:0x%08llX",
                       code);
        return false;
    }

    code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                            DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                            NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic avoid data failed,error code:0x%08llX",
                       code);
        return false;
    }

    code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);
    if (code != DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE &&
        code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic quaternion failed,error code:0x%08llX",
                       code);
        return false;
    }

    code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                            DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                            NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic position fused failed,error code:0x%08llX",
                       code);
        return false;
    }

    code = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                            DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                            NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude fused failed,error code:0x%08llX",
                       code);
        return false;
    }

    code =
    DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                     DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                     NULL);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude of home point failed,"
                       "error code:0x%08llX", code);
        return false;
    }
    cout << "flight module has been initialized" << endl;
    return true;
}

bool FlightModule::setAndCheckDroneSafeParameters()
{
    T_DjiReturnCode code;

    // Turn on horizontal vision avoidance
    code = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
                                     DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on horizontal visual obstacle avoidance "
                       "failed, error code: 0x%08X", code);
        return false;
    };
    millisecond_delay(1000);
    E_DjiFlightControllerObstacleAvoidanceEnableStatus
             horizontalVisualObstacleAvoidanceStatus;
    code = DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(
                                             &horizontalVisualObstacleAvoidanceStatus);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get horizontal visual obstacle avoidance failed, "
                       "error code: 0x%08X", code);
        return false;
    }
    cout << "Current horizontal visual obstacle avoidance status is: "
         << horizontalVisualObstacleAvoidanceStatus << endl;

    // Turn on horizontal radar avoidance
    code = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
                                     DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on horizontal radar obstacle avoidance "
                       "failed, error code: 0x%08X", code);
        return false;
    };
    millisecond_delay(1000);
    E_DjiFlightControllerObstacleAvoidanceEnableStatus
            horizontalRadarObstacleAvoidanceStatus;
    code = DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(
                                            &horizontalRadarObstacleAvoidanceStatus);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get horizontal radar obstacle avoidance failed, "
                       "error code: 0x%08X", code);
        return false;
    }
    cout << "Current horizontal radar obstacle avoidance status is: "
         << horizontalRadarObstacleAvoidanceStatus << endl;

    // Disable upwards vision avoidance
    code = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
                                      DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Disable upwards visual obstacle avoidance failed, "
                       "error code: 0x%08X", code);
        return false;
    };
    millisecond_delay(1000);
    E_DjiFlightControllerObstacleAvoidanceEnableStatus
            upwardsVisualObstacleAvoidanceStatus;
    code = DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
                                             &upwardsVisualObstacleAvoidanceStatus);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get upwards visual obstacle avoidance failed, "
                       "error code: 0x%08X", code);
        return false;
    }
    cout << "Current upwards visual obstacle avoidance status is: "
         << upwardsVisualObstacleAvoidanceStatus << endl;

    // Disable upwards radar avoidance
    code = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
                                     DJI_FLIGHT_CONTROLLER_DISABLE_OBSTACLE_AVOIDANCE);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Disable upwards radar obstacle avoidance failed, "
                       "error code: 0x%08X", code);
        return false;
    }
    millisecond_delay(1000);
    E_DjiFlightControllerObstacleAvoidanceEnableStatus
            upwardsRadarObstacleAvoidanceStatus;
    code = DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(
                                             &upwardsRadarObstacleAvoidanceStatus);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get upwards radar obstacle avoidance failed, "
                       "error code: 0x%08X", code);
        return false;
    }
    cout << "Current upwards radar obstacle avoidance status is:"
         << upwardsRadarObstacleAvoidanceStatus << endl;

    // Turn on downward vision avoidance
    code = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
                                      DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on downwards visual obstacle avoidance "
                       "failed, error code: 0x%08X", code);
        return false;
    }
    millisecond_delay(1000);
    E_DjiFlightControllerObstacleAvoidanceEnableStatus
            downloadsVisualObstacleAvoidanceStatus;
    code = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(
                                            &downloadsVisualObstacleAvoidanceStatus);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get downwards visual obstacle avoidance failed, "
                       "error code: 0x%08X", code);
        return false;
    }
    cout << "Current downwards visual obstacle avoidance status is:"
         << downloadsVisualObstacleAvoidanceStatus << endl;

    // Set a go-home altitude
    code = DjiFlightController_SetGoHomeAltitude(50);  // in meter.
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set go home altitude to 50(m) failed, error code: 0x%08X", code);
        return false;
    }
    millisecond_delay(1000);
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    code = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", code);
        return false;
    }
    cout << "Current go home altitude is:" << goHomeAltitude << endl;

    // disable rtk
    code = DjiFlightController_SetRtkPositionEnableStatus(
                                DJI_FLIGHT_CONTROLLER_DISABLE_RTK_POSITION);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Disable rtk failed, error code: 0x%08X", code);
        return false;
    }
    millisecond_delay(1000);

    // Set RC-loss action */
    code = DjiFlightController_SetRCLostAction(
                         DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_GOHOME);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set rc lost action failed, error code: 0x%08X", code);
        return false;
    }
    millisecond_delay(1000);
    E_DjiFlightControllerRCLostAction rcLostAction;
    code = DjiFlightController_GetRCLostAction(&rcLostAction);
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get rc lost action failed, error code: 0x%08X", code);
        return false;
    }
    cout << "Current rc-loss action is: " << rcLostAction <<" "
         << "(Go-home is 2) " << endl;
    return true;
}

bool FlightModule::cleanup()
{
    T_DjiReturnCode code = DjiFcSubscription_DeInit();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                       code);
        return false;
    }

    code = DjiFlightController_DeInit();
    if (code != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                       code);
        return false;
    }

    return true;
}
