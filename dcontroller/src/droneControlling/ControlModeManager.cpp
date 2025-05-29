#include <cassert>
#include <iostream>
#include "options.h"
#include "ControlModeManager.h"

using namespace std;

ControlModeManager::ControlModeManager()
{
  instructions["D1"] = D1;
  instructions["D2"] = D2;
  instructions["D3"] = D3;
  instructions["D4"] = D4;
  instructions["D5"] = D5;
  instructions["D6"] = D6;
  instructions["D7"] = D7;
  instructions["D8"] = D8;
  instructions["D9"] = D9;
  instructions["D10"] = D10;
  instructions["D11"] = D11;
  instructions["D13"] = D13;
  instructions["D21"] = D21;
  instructions["D22"] = D22;
  instructions["D23"] = D23;
  instructions["D24"] = D24;
  instructions["D25"] = D25;
  instructions["D26"] = D26;
  instructions["D29"] = D29;
  instructions["D30"] = D30;
  instructions["D31"] = D31;
}

// runtime singleton
ControlModeManager *ControlModeManager::singleton = nullptr;

T_DjiOsalHandler *ControlModeManager::s_osalHandler = nullptr;

int cruisePointCount = 0; // 当前巡航点数量，全局变量

ControlModeManager::~ControlModeManager()
{
  T_DjiReturnCode returnCode;
  returnCode = FlightControlDeInit();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                   returnCode);
    // 继续执行，确保所有资源都被释放
  }
}

bool ControlModeManager::canProcessInstruction(const string &instructionID)
{
  return instructions.count(instructionID) > 0;
}

// void ControlModeManager::emergencyStop()
// {
//   // switchControlMode(InstructionFollowingMode);
// }

Waypoint ControlModeManager::getCurrentPosition()
{
  Waypoint pos;
  T_DjiFcSubscriptionGpsPosition gpsPosition;
  T_DjiReturnCode ret = DjiFcSubscription_GetLatestValueOfTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
      (uint8_t *)&gpsPosition,
      sizeof(T_DjiFcSubscriptionGpsPosition),
      NULL);

  if (ret == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    pos.latitude = gpsPosition.y / 1e7;
    pos.longitude = gpsPosition.x / 1e7;
    pos.altitude = gpsPosition.z / 1000.0;
  }
  else
  {
    // 错误处理，设为0
    pos.latitude = 0;
    pos.longitude = 0;
    pos.altitude = 0;
  }
  return pos;
}

static T_DjiReturnCode
DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
  switch (eventData.joystickCtrlAuthoritySwitchEvent)
  {
  case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT:
  {
    if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK)
    {
      USER_LOG_INFO("[Event] Msdk request to obtain joystick ctrl authority\r\n");
    }
    else
    {
      USER_LOG_INFO("[Event] Msdk request to release joystick ctrl authority\r\n");
    }
    break;
  }
  case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT:
  {
    if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL)
    {
      USER_LOG_INFO("[Event] Internal request to obtain joystick ctrl authority\r\n");
    }
    else
    {
      USER_LOG_INFO("[Event] Internal request to release joystick ctrl authority\r\n");
    }
    break;
  }
  case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT:
  {
    if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK)
    {
      USER_LOG_INFO("[Event] Request to obtain joystick ctrl authority\r\n");
    }
    else
    {
      USER_LOG_INFO("[Event] Request to release joystick ctrl authority\r\n");
    }
    break;
  }
  case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc lost\r\n");
    break;
  case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
    break;
  case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
    break;
  case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
    break;
  case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
    break;
  case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for low battery return\r\n");
    break;
  case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc for low battery land\r\n");
    break;
  case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to sdk lost\r\n");
    break;
  case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT:
    USER_LOG_INFO("[Event] Current joystick ctrl authority is reset to rc due to near boundary\r\n");
    break;
  default:
    USER_LOG_INFO("[Event] Unknown joystick ctrl authority event\r\n");
  }
  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode ControlModeManager::initialize()
{
  T_DjiReturnCode returnCode;
  T_DjiFlightControllerRidInfo ridInfo = {0};

  s_osalHandler = DjiPlatform_GetOsalHandler();
  if (!s_osalHandler)
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

  // 1. 初始化订阅模块
  returnCode = DjiFcSubscription_Init();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX", returnCode);
    return returnCode;
  }

  // 2. 订阅GPS话题
  returnCode = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
      DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ,
      NULL);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Subscribe topic gps failed, error code:0x%08llX", returnCode);
    return returnCode;
  }

  // 3. 等待获取有效GPS数据（循环获取直到成功或超时）
  T_DjiFcSubscriptionGpsPosition gpsPosition;
  int tryCount = 0;
  const int MAX_RETRY_COUNT = 100;  // 最多等待5秒 (100 * 50ms = 5000ms)
  const int RETRY_INTERVAL_MS = 50; // 每次重试间隔50ms
  bool gpsValid = false;

  while (tryCount++ < MAX_RETRY_COUNT)
  {
    T_DjiReturnCode gpsRet = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        (uint8_t *)&gpsPosition,
        sizeof(T_DjiFcSubscriptionGpsPosition),
        NULL);

    // 判断有效的纬度经度和返回码
    if (gpsRet == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        gpsPosition.y != 0 && gpsPosition.x != 0)
    {
      gpsValid = true;
      break;
    }
    s_osalHandler->TaskSleepMs(RETRY_INTERVAL_MS);
  }

  if (!gpsValid)
  {
    USER_LOG_ERROR("Failed to get valid GPS position within timeout period");
    return DJI_ERROR_SYSTEM_MODULE_CODE_TIMEOUT;
  }

  // 4. 用实时GPS数据初始化ridInfo
  ridInfo.latitude = gpsPosition.y / 10000000.0;
  ridInfo.longitude = gpsPosition.x / 10000000.0;
  ridInfo.altitude = gpsPosition.z;

  // 5. 初始化飞控
  returnCode = DjiFlightController_Init(ridInfo);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX", returnCode);
    return returnCode;
  }

  // 6. 继续订阅其他话题
  struct
  {
    E_DjiFcSubscriptionTopic topic;
    E_DjiDataSubscriptionTopicFreq freq;
    const char *name;
  } topics[] = {
      {DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "flight status"},
      {DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "display mode"},
      {DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "height fusion"},
      {DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, "quaternion"},
      {DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, "position fused"},
      {DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ, "altitude fused"},
      {DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT, DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ, "altitude of home point"},
      {DJI_FC_SUBSCRIPTION_TOPIC_POSITION_VO, DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ, "position vo"},
      {DJI_FC_SUBSCRIPTION_TOPIC_CONTROL_DEVICE, DJI_DATA_SUBSCRIPTION_TOPIC_5_HZ, "control device"},
  };

  for (size_t i = 0; i < sizeof(topics) / sizeof(topics[0]); ++i)
  {
    returnCode = DjiFcSubscription_SubscribeTopic(
        topics[i].topic,
        topics[i].freq,
        NULL);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        !(topics[i].topic == DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION && returnCode == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE))
    {
      USER_LOG_ERROR("Subscribe topic %s failed, error code:0x%08llX", topics[i].name, returnCode);
      return returnCode;
    }
  }

  // 7. 注册遥控权回调
  returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
      DjiUser_FlightCtrlJoystickCtrlAuthSwitchEventCb);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT)
  {
    USER_LOG_ERROR("Register joystick control authority event callback failed, error code:0x%08llX", returnCode);
    return returnCode;
  }

  T_DjiFlightControllerGeneralInfo generalInfo = {0};
  returnCode = DjiFlightController_GetGeneralInfo(&generalInfo);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    USER_LOG_ERROR("Get general info failed, error code:0x%08llX", returnCode);
    return returnCode;
  }
  USER_LOG_INFO("Get aircraft serial number is: %s", generalInfo.serialNum);

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode ControlModeManager::FlightControlDeInit()
{
  T_DjiReturnCode returnCode;

  // 1. 先反初始化订阅模块
  returnCode = DjiFcSubscription_DeInit();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                   returnCode);
    return returnCode;
  }

  // 2. 再反初始化飞控
  returnCode = DjiFlightController_DeInit();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                   returnCode);
    return returnCode;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

bool ControlModeManager::FlightTakeoffAndLanding(int ID)
{
  T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    return false;
  }
  s_osalHandler->TaskSleepMs(1000);

  switch (ID)
  {
  case 1: // 起飞
  {
    // 1. 开始起飞
    returnCode = DjiFlightController_StartTakeoff();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      USER_LOG_ERROR("Take off failed, error code: 0x%08llX", returnCode);
      return false;
    }

    // 2. 检查电机是否启动
    if (!MotorStartedCheck())
    {
      USER_LOG_ERROR("Takeoff failed. Motors are not spinning.");
      return false;
    }
    USER_LOG_INFO("Motors spinning...");

    // 3. 检查是否离地
    if (!TakeOffInAirCheck())
    {
      USER_LOG_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning");
      return false;
    }
    USER_LOG_INFO("Ascending...");

    // 4. 检查起飞是否完成
    if (!TakeoffFinishedCheck())
    {
      USER_LOG_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
      return false;
    }
    USER_LOG_INFO("Successful take off");
    return true;
  }
  case 2: // 降落
  {
    // 1. 开始降落
    returnCode = DjiFlightController_StartLanding();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Landing failed, error code: 0x%08llX", returnCode);
        return false;
    }

    // 2. 检查降落是否开始
    if (!CheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING))
    {
        USER_LOG_ERROR("Fail to execute Landing action!");
        return false;
    }

    // 3. 检查降落是否完成
    if (!LandFinishedCheck())
    {
        USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. Please connect DJI Assistant.");
        return false;
    }

    USER_LOG_INFO("Successful landing");
    return true;
  }
  default:
    return false;
  }
}

// 检查电机是否成功启动，超时时间2秒
bool ControlModeManager::MotorStartedCheck()
{
  int motorsNotStarted = 0;
  int timeoutCycles = 20;  // 2秒超时 (20 * 100ms)

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
      USER_LOG_ERROR("Get value of topic flight status error, error code: 0x%08X", returnCode);
      return false;
    }

    // 检查是否满足启动条件
    if (flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND ||
        displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START) {
      return true;
    }

    motorsNotStarted++;
    if (motorsNotStarted >= timeoutCycles) {
      return false;
    }
    
    s_osalHandler->TaskSleepMs(100);
  }
}

// 检查无人机是否成功离地，超时时间11秒
bool ControlModeManager::TakeOffInAirCheck()
{
  int stillOnGround = 0;
  int timeoutCycles = 110;  // 11秒超时 (110 * 100ms)

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
      USER_LOG_ERROR("Get value of topic flight status error, error code: 0x%08X", returnCode);
      return false;
    }

    // 检查是否满足离地条件
    if (flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR ||
        (displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
         displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF)) {
      return true;
    }

    stillOnGround++;
    if (stillOnGround >= timeoutCycles) {
      return false;
    }
    
    s_osalHandler->TaskSleepMs(100);
  }
}

// 检查起飞是否完成并进入正常飞行模式
bool ControlModeManager::TakeoffFinishedCheck()
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
            s_osalHandler->TaskSleepMs(1000);
            continue;
        }

        // 检查是否进入正常飞行模式
        return (displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
                displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE);
    }
}

// 检查降落是否完成
bool ControlModeManager::LandFinishedCheck()
{
    while (true) {
        T_DjiFcSubscriptionDisplaymode displayMode = {0};
        T_DjiFcSubscriptionFlightStatus flightStatus = {0};
        
        T_DjiReturnCode returnCode = DjiFcSubscription_GetLatestValueOfTopic(
            DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
            (uint8_t *)&displayMode,
            sizeof(T_DjiFcSubscriptionDisplaymode),
            NULL);
            
        if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            returnCode = DjiFcSubscription_GetLatestValueOfTopic(
                DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                (uint8_t *)&flightStatus,
                sizeof(T_DjiFcSubscriptionFlightStatus),
                NULL);
        }

        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            return false;
        }

        // 如果仍在降落模式或仍在空中，继续等待
        if (displayMode == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING ||
            flightStatus == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            s_osalHandler->TaskSleepMs(1000);
            continue;
        }

        // 检查是否已降落（不在P-GPS或ATTITUDE模式）
        return (displayMode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS &&
                displayMode != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE);
    }
}

// 检查动作是否开始
bool ControlModeManager::CheckActionStarted(E_DjiFcSubscriptionDisplayMode mode)
{
    int actionNotStarted = 0;
    int timeoutCycles = 20;  // 2秒超时

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

        if (displayMode == mode) {
            USER_LOG_INFO("Now flight is in mode: %d", mode);
            return true;
        }

        actionNotStarted++;
        if (actionNotStarted >= timeoutCycles) {
            USER_LOG_ERROR("Action start failed, current mode: %d, expected mode: %d", 
                          displayMode, mode);
            return false;
        }
        
        s_osalHandler->TaskSleepMs(100);
    }
}

void ControlModeManager::FlightControl_VelocityControl(T_DjiFlightControllerJoystickCommand command)
{
    // 速度限制检查
    if (command.x > 10.0 || command.y > 10.0 || command.z > 10.0)
    {
        USER_LOG_ERROR("Speed command exceeds limit");
        return;
    }

    // 获取控制权限
    T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        return;
    }
    s_osalHandler->TaskSleepMs(1000);

    // 设置控制模式
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(joystickMode);

    // 停止之前的控制
    stopControl();  // 确保之前的控制已停止

    // 设置新的控制命令
    currentCommand = command;
    isControlling = true;

    // 启动新的控制线程
    if (controlThread.joinable()) {
        controlThread.join();  // 等待之前的线程结束
    }
    controlThread = std::thread(&ControlModeManager::controlLoop, this);
}

void ControlModeManager::FlightControl_setPitchAndYaw(T_DjiFlightControllerJoystickCommand command)
{
    // 角度限制检查
    if (command.x > 30.0f || command.x < -10.0f || command.yaw > 180.0f || command.yaw < -180.0f)
    {
        USER_LOG_ERROR("Angle command exceeds limit");
        return;
    }

    // 获取控制权限
    T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        return;
    }
    s_osalHandler->TaskSleepMs(1000);

    // 设置控制模式
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(joystickMode);

    // 停止之前的控制
    stopControl();  // 确保之前的控制已停止

    // 设置新的控制命令
    currentCommand = command;
    isControlling = true;

    // 启动新的控制线程
    if (controlThread.joinable()) {
        controlThread.join();  // 等待之前的线程结束
    }
    controlThread = std::thread(&ControlModeManager::controlLoop, this);
}

void ControlModeManager::setForwardAcceleration(float acceleration, float maxSpeed = 5.0f, uint32_t durationMs = 10 * 1000)
{
  float speed = 0;
  float dt = 0.05f; // 控制周期，单位：秒
  uint32_t stepMs = dt * 1000;
  uint32_t elapsed = 0;

  T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    return;
  }
  s_osalHandler->TaskSleepMs(1000);

  // 设置为速度控制模式
  T_DjiFlightControllerJoystickMode joystickMode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
  };
  DjiFlightController_SetJoystickMode(joystickMode);

  while (elapsed < durationMs)
  {
    speed += acceleration * dt;
    if (speed >= maxSpeed)
      speed = maxSpeed;

    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = speed; // 向前速度
    DjiFlightController_ExecuteJoystickAction(cmd);

    s_osalHandler->TaskSleepMs(stepMs);
    elapsed += stepMs;
  }
  // 达到最大速度后保持最大速度前进
}

void ControlModeManager::emergencyBrake() // 急停，优先级高
{
    // 停止控制循环
    stopControl();

    T_DjiReturnCode ret = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", ret);
        return;
    }

    ret = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", ret);
    }

    s_osalHandler->TaskSleepMs(1000);

    ret = DjiFlightController_CancelEmergencyBrakeAction();
    if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", ret);
    }
}

void ControlModeManager::hover() // 悬停而非急停，优先级可能较低，暂未使用
{
  T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    return;
  }
  s_osalHandler->TaskSleepMs(1000);

  T_DjiFlightControllerJoystickMode joystickMode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
  };
  DjiFlightController_SetJoystickMode(joystickMode);

  T_DjiFlightControllerJoystickCommand cmd = {0};
  cmd.x = 0;
  cmd.y = 0;
  cmd.z = 0;
  cmd.yaw = 0;
  DjiFlightController_ExecuteJoystickAction(cmd);
}

// 计算两点之间的球面距离（单位：米）
double ControlModeManager::calcDistance(const Waypoint &p1, const Waypoint &p2)
{
  double lat1 = p1.latitude * M_PI / 180.0;
  double lon1 = p1.longitude * M_PI / 180.0;
  double lat2 = p2.latitude * M_PI / 180.0;
  double lon2 = p2.longitude * M_PI / 180.0;

  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;

  double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
             std::cos(lat1) * std::cos(lat2) *
                 std::sin(dlon / 2) * std::sin(dlon / 2);
  double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
  return EARTH_RADIUS * c;
}

// 计算从p1指向p2的方位角（弧度，北为0，东为正pi/2）
double ControlModeManager::calcHeading(const Waypoint &p1, const Waypoint &p2)
{
  double lat1 = p1.latitude * M_PI / 180.0;
  double lon1 = p1.longitude * M_PI / 180.0;
  double lat2 = p2.latitude * M_PI / 180.0;
  double lon2 = p2.longitude * M_PI / 180.0;
  double dlon = lon2 - lon1;
  double y = std::sin(dlon) * std::cos(lat2);
  double x = std::cos(lat1) * std::sin(lat2) -
             std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
  return std::atan2(y, x); // 范围[-pi, pi]
}

/// 飞行到目标点，速度和到达阈值可选，默认速度2米每秒，到达阈值1米。到达阈值指的是与目标点的距离小于该值时，认为到达目标点。
void ControlModeManager::flyToTarget(const Waypoint &target, float speed = 2.0f, float arriveThresh = 1.0f)
{
  T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    return;
  }

  T_DjiFlightControllerJoystickMode joystickMode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystickMode);

  while (true)
  {
    Waypoint curr = getCurrentPosition();
    double dist = calcDistance(curr, target);
    if (dist < arriveThresh)
    {
      break;
    }

    T_DjiFlightControllerJoystickCommand cmd = {0};
    double heading = calcHeading(curr, target);
    cmd.x = speed * std::cos(heading);
    cmd.y = speed * std::sin(heading);
    double dAlt = target.altitude - curr.altitude;
    double k_alt = 0.5;
    double v_z = k_alt * dAlt;
    const double maxVz = 2.0;
    if (v_z > maxVz)
      v_z = maxVz;
    if (v_z < -maxVz)
      v_z = -maxVz;
    cmd.z = v_z;
    cmd.yaw = 0.0f;

    DjiFlightController_ExecuteJoystickAction(cmd);
    s_osalHandler->TaskSleepMs(100);
  }
  // 任务结束，无论什么原因，都下发0速度悬停
  T_DjiFlightControllerJoystickCommand stopCmd = {0};
  DjiFlightController_ExecuteJoystickAction(stopCmd);
}

std::vector<Waypoint> getWaypoints(const Json::Value &instructionParameter)
{
  std::vector<Waypoint> waypoints;
  cruisePointCount = 0;
  const Json::Value &arr = instructionParameter["waypoints"];
  if (!arr.isNull() && arr.isArray())
  {
    // 多点输入
    for (Json::ArrayIndex i = 0; i < arr.size(); ++i)
    {
      Waypoint wp;
      wp.latitude = arr[i]["latitude"].asDouble();
      wp.longitude = arr[i]["longitude"].asDouble();
      wp.altitude = arr[i]["altitude"].asDouble();
      waypoints.push_back(wp);
      cruisePointCount++;
    }
  }
  else
  {
    // 单点输入
    Waypoint wp;
    wp.latitude = instructionParameter["latitude"].asDouble();
    wp.longitude = instructionParameter["longitude"].asDouble();
    wp.altitude = instructionParameter["altitude"].asDouble();
    waypoints.push_back(wp);
    cruisePointCount = 1;
  }
  return waypoints;
}

// D8巡航
void ControlModeManager::cruisePath(float speed, float arriveThresh)
{
  if (cruisePointCount < 1 || cruisePoints.empty())
  {
    USER_LOG_ERROR("No cruise points set or cruise points array is empty.\r\n");
    return;
  }
  else
  {
    for (int i = 0; i < cruisePointCount && i < cruisePoints.size(); ++i)
    {
      flyToTarget(cruisePoints[i], speed, arriveThresh);
    }
  }
}

// D23飞行到目标点
void ControlModeManager::goToTargetPoint(const Waypoint &target, float speed, float arriveThresh)
{
  flyToTarget(target, speed, arriveThresh);
}

void ControlModeManager::processInstruction(const Json::Value &instructionData)
{
  string instructionID = instructionData["instructionID"].asString();

  // 检查指令ID是否存在
  if (instructions.find(instructionID) == instructions.end())
  {
    USER_LOG_ERROR("Unknown instruction ID: %s", instructionID.c_str());
    return;
  }

  InstructionNames instruction = instructions[instructionID];

  switch (instruction)
  {
  case D1:
  {
    USER_LOG_INFO("Processing D1: Takeoff");
    FlightTakeoffAndLanding(1);
    break;
  }
  case D2:
  {
    USER_LOG_INFO("Processing D2: Landing");
    FlightTakeoffAndLanding(2);
    break;
  }
  case D3: // 前进
  {
    USER_LOG_INFO("Processing D3: Move forward");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = 0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D4: // 后退
  {
    USER_LOG_INFO("Processing D4: Move backward");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = -0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D5: // 向左
  {
    USER_LOG_INFO("Processing D5: Move left");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.y = -0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D6: // 向右
  {
    USER_LOG_INFO("Processing D6: Move right");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.y = 0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D7:
  {
    switchControlMode(HangingMode);
    break;
  }
  case D8: // 巡航
  {
    USER_LOG_INFO("Processing D8: Cruise");
    float speed = instructionData["instructionParameter"]["speed"].asFloat();
    float arriveThresh = instructionData["instructionParameter"]["arriveThresh"].asFloat();

    // 不再从指令参数中获取路径点，而是使用已保存的路径点
    if (cruisePoints.empty())
    {
      USER_LOG_ERROR("No waypoints saved for cruise");
      return;
    }

    cruisePath(speed, arriveThresh);
    break;
  }
  case D9: // 设置前进速度
  {
    USER_LOG_INFO("Processing D9: Set speed");
    float speed = instructionData["instructionParameter"].asFloat();
    if (speed > 5.0f)
      speed = 5.0f;
    if (speed < 0.0f)
      speed = 0.0f; // 限制速度范围
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = speed;
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D10: // 设置前进加速度
  {
    USER_LOG_INFO("Processing D10: Set acceleration");
    float acceleration = instructionData["instructionParameter"]["acceleration"].asFloat();
    float maxSpeed = instructionData["instructionParameter"]["maxSpeed"].asFloat();
    if (acceleration > 0.5f)
      acceleration = 0.5f;
    if (acceleration < 0.0f)
      acceleration = 0.0f; // 限制加速度范围

    setForwardAcceleration(acceleration, maxSpeed);
    break;
  }
  case D11:
  {
    USER_LOG_INFO("Processing D11: Set flight pitch");
    float pitchAngle = instructionData["instructionParameter"].asFloat();
    if (pitchAngle > 30.0f)
      pitchAngle = 30.0f;
    if (pitchAngle < -10.0f)
      pitchAngle = -10.0f; // 限制俯仰角范围
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = pitchAngle;
    FlightControl_setPitchAndYaw(cmd);
    break;
  }
  case D13:
  {
    USER_LOG_INFO("Processing D13: Set flight yaw");
    float yawAngle = instructionData["instructionParameter"].asFloat();
    if (yawAngle > 180.0f)
      yawAngle = 180.0f;
    if (yawAngle < -180.0f)
      yawAngle = -180.0f; // 限制航向角范围
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.yaw = yawAngle;
    FlightControl_setPitchAndYaw(cmd);
    break;
  }
  case D21: // 上升
  {
    USER_LOG_INFO("Processing D21: Ascend");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.z = -0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D22: // 下降
  {
    USER_LOG_INFO("Processing D22: Descend");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.z = 0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D23: // 前往目标位置（下一个目标点）
  {
    USER_LOG_INFO("Processing D23: Go to target point");

    if (cruisePoints.empty())
    {
      USER_LOG_ERROR("No waypoints saved for target point");
      return;
    }

    float speed = instructionData["instructionParameter"]["speed"].asFloat();
    float arriveThresh = instructionData["instructionParameter"]["arriveThresh"].asFloat();

    goToTargetPoint(cruisePoints[0], speed, arriveThresh);
    break;
  }
  case D24: // 向左前方10°方向前进
  {
    USER_LOG_INFO("Processing D24: Turn left");
    float angle = 10.0f * M_PI / 180.0f; // 弧度
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = 0.5 * cos(angle);
    cmd.y = -0.5 * sin(angle);
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D25: // 向右前方10°方向前进
  {
    USER_LOG_INFO("Processing D23: Turn right");
    float angle = -10.0f * M_PI / 180.0f;
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = 0.5 * cos(angle);
    cmd.y = 0.5 * sin(angle);
    FlightControl_VelocityControl(cmd);
    break;
  }
  case D26: // 返航
  {
    USER_LOG_INFO("Processing D26: Return to home");
    T_DjiReturnCode returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      USER_LOG_ERROR("Return to home failed, error code: 0x%08X", returnCode);
    }
    break;
  }
  case D29: // 保存路径点
  {
    Waypoint currentPos = getCurrentPosition();
    saveWaypoint(currentPos);
    break;
  }
  case D30: // 设置返航点
  {
    Waypoint currentPos = getCurrentPosition();
    setHomePoint(currentPos);
    break;
  }
  case D31: // 清除所有路径点
  {
    clearAllWaypoints();
    break;
  }
  }
}

void ControlModeManager::switchControlMode(ControlMode newMode)
{
  // For some modes, before leaving old mode, some cleanup-work
  // should be done.
  controlMode = newMode;
  switch (controlMode)
  {
  case HangingMode:
  {
    emergencyBrake();
    break;
  }
  case UnspecifiedMode:
    break;
  }
}

void ControlModeManager::appendControllerStatus(Json::Value &status)
{
  //    // No matter in which control mode,  append soldier position if it is available.
  //    SoldierFollowerController* soldierFollowerController = SoldierFollowerController::get();
  //    PositionRecord position;
  //    if ( soldierFollowerController->getCurrentPosition(position) ){
  //        status["soldierPositionX"] = position.x;
  //        status["soldierPositionY"] = position.y;
  //    }

  //    switch (controlMode){
  //      case UGVDisabled:
  //      case UnspecifiedMode:
  //      case InstructionFollowingMode:
  //          break;
  //      case AccelerationControlMode:{
  //          AccelerationController* accelerationController = AccelerationController::get();
  //          status["recentAcceleration"] = accelerationController->getRecentAccelerationAverage();
  //          break;
  //      }
  //      case AngularSpeedControlMode:{
  //          AngularSpeedController* angularSpeedController = AngularSpeedController::get();
  //          status["angularSpeed"] = angularSpeedController->getAngularSpeed();
  //          break;
  //      }
  //      case SoldierFollowingMode:
  //          break;
  //      case YawTargetingMode:{
  //          YawController* yawController = YawController::get();
  //          double yaw;
  //          if (yawController->getCurrentYawInUnidirectionConvention(yaw)){
  //            status["ugvYaw"] = yaw;
  //          }
  //          break;
  //      }
  //   }
}

ControlMode ControlModeManager::getControlMode() const
{
  return controlMode;
}

ControlModeManager *ControlModeManager::createInstance()
{
  assert(singleton == nullptr);
  singleton = new ControlModeManager();
  return singleton;
}

ControlModeManager *ControlModeManager::get()
{
  assert(singleton);
  return singleton;
}

// 保存路径点
void ControlModeManager::saveWaypoint(const Waypoint &waypoint)
{
  cruisePoints.push_back(waypoint);
  USER_LOG_INFO("保存路径点成功，当前共有 %zu 个路径点", cruisePoints.size());
}

// 设置返航点
void ControlModeManager::setHomePoint(const Waypoint &waypoint)
{
  homePoint = waypoint;
  USER_LOG_INFO("设置返航点成功，位置：经度=%.6f，纬度=%.6f，高度=%.2f",
                waypoint.longitude, waypoint.latitude, waypoint.altitude);
}

// 清除所有保存的路径点
void ControlModeManager::clearAllWaypoints()
{
  cruisePoints.clear();
  USER_LOG_INFO("已清除所有路径点");
}

// 获取所有保存的路径点
const std::vector<Waypoint> &ControlModeManager::getSavedWaypoints() const
{
  return cruisePoints;
}

// 获取返航点
const Waypoint &ControlModeManager::getHomePoint() const
{
  return homePoint;
}

void ControlModeManager::controlLoop()
{
    USER_LOG_INFO("Control loop started");
    while (isControlling) {
        T_DjiReturnCode returnCode = DjiFlightController_ExecuteJoystickAction(currentCommand);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
        {
            USER_LOG_ERROR("Execute joystick command failed, errno = 0x%08llX", returnCode);
            isControlling = false;
            return;
        }
        s_osalHandler->TaskSleepMs(2);
    }
    USER_LOG_INFO("Control loop stopped");
}

void ControlModeManager::stopControl()
{
    if (!isControlling) {
        return;  // 如果已经停止，直接返回
    }
    
    USER_LOG_INFO("Stopping control loop");
    isControlling = false;
    if (controlThread.joinable()) {
        controlThread.join();
    }
    USER_LOG_INFO("Control loop stopped");
}
