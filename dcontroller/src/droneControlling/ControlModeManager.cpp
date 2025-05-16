#include <cassert>
#include <iostream>
#include "options.h"
#include "ControlModeManager.h"
#include "../gpioManipulators/FanController.h"
#include "../gpioManipulators/LightController.h"

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
  instructions["D27"] = D27;
  instructions["D28"] = D28;
}

// runtime singleton
ControlModeManager *ControlModeManager::singleton = nullptr;

ControlModeManager::~ControlModeManager()
{
  T_DjiReturnCode returnCode;
  returnCode = FlightControlDeInit();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                   returnCode);
    return;
  }
}

bool ControlModeManager::canProcessInstruction(const string &instructionID)
{
  return instructions.count(instructionID) > 0;
}

void ControlModeManager::emergencyStop()
{
  // switchControlMode(InstructionFollowingMode);
}

Waypoint ControlModeManager::getCurrentPosition()
{
  Waypoint pos;
  _FlightControllerGpsInfo gpsInfo;
  DjiFlightController_GetGpsInfo(&gpsInfo, 1000);
  pos.latitude = gpsInfo.latitude / 1e7;   // 纬度
  pos.longitude = gpsInfo.longitude / 1e7; // 经度
  pos.altitude = gpsInfo.altitude;         // 高度
  // pos.altitude = DjiUser_FlightControlGetValueOfRelativeHeight(); // 相对高度
  return pos;
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
  const int tryMax = 100; // 最多等5秒
  bool gpsValid = false;
  while (tryCount++ < tryMax)
  {
    T_DjiReturnCode gpsRet = DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
        (uint8_t *)&gpsPosition,
        sizeof(T_DjiFcSubscriptionGpsPosition),
        NULL);
    // 判断有效的纬度经度和返回码
    if (gpsRet == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
        gpsPosition.latitude != 0 && gpsPosition.longitude != 0)
    {
      gpsValid = true;
      break;
    }
    s_osalHandler->TaskSleepMs(50); // 等50ms再试
  }
  if (!gpsValid)
  {
    USER_LOG_ERROR("Failed to get valid GPS position for RID info");
    return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;
  }

  // 4. 用实时GPS数据初始化ridInfo
  ridInfo.latitude = gpsPosition.latitude / 10000000.0;
  ridInfo.longitude = gpsPosition.longitude / 10000000.0;
  ridInfo.altitude = gpsPosition.altitude;

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
      DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS &&
      returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT)
  {
    USER_LOG_ERROR("Register joystick control authority event callback failed, error code:0x%08llX", returnCode);
    return returnCode;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode FlightControlDeInit(void)
{
  T_DjiReturnCode returnCode;

  returnCode = DjiFcSubscription_DeInit();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                   returnCode);
    return returnCode;
  }

  returnCode = DjiFlightController_DeInit();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                   returnCode);
    return returnCode;
  }

  return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void ControlModeManager::FlightTakeoffAndLanding(int ID)
{
  T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    return;
  }
  s_osalHandler->TaskSleepMs(1000);
  switch (ID)
  {
  case 1:
    if (!DjiTest_FlightControlMonitoredTakeoff())
    {
      USER_LOG_ERROR("Take off failed");
      return;
    }
    USER_LOG_INFO("Successful take off\r\n");
  case 2:
    if (!DjiTest_FlightControlMonitoredLanding())
    {
      USER_LOG_ERROR("Landing failed");
      return;
    }
    USER_LOG_INFO("Successful landing\r\n");
    break;
  default:
    break;
  }
}

void ControlModeManager::FlightControl_VelocityControl(T_DjiFlightControllerJoystickCommand command)
{
  T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    return;
  }
  s_osalHandler->TaskSleepMs(1000);

  T_DjiFlightControllerJoystickMode joystickMode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE, // 水平速度控制
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,   // 垂直速度控制
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,      // 航向角速度控制
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,       // 无人机本体为坐标系
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,       // 稳定控制模式
  };
  DjiFlightController_SetJoystickMode(joystickMode);
  USER_LOG_DEBUG("Joystick command: %.2f %.2f %.2f", command.x, command.y, command.z);
  returnCode = DjiFlightController_ExecuteJoystickAction(command);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Execute joystick command failed, errno = 0x%08llX", returnCode);
    return;
  }
}

void ControlModeManager::FlightControl_setPitchAndYaw(T_DjiFlightControllerJoystickCommand command)
{
  T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
    return;
  }
  s_osalHandler->TaskSleepMs(1000);

  T_DjiFlightControllerJoystickMode joystickMode = {
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
      DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
      DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE};
  DjiFlightController_SetJoystickMode(joystickMode);
  USER_LOG_DEBUG("Joystick command: %.2f %.2f %.2f", command.x, command.y, command.z);
  returnCode = DjiFlightController_ExecuteJoystickAction(command);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
  {
    USER_LOG_ERROR("Execute joystick command failed, errno = 0x%08llX", returnCode);
    return;
  }
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
  const Json::Value &arr = instructionParameter["waypoints"];
  cruisePointCount = 0;
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
  if (cruisePointCount < 1)
  {
    USER_LOG_ERROR("No cruise points set.\r\n");
    return;
  }
  else
  {
    for (int i = 0; i < cruisePointCount && !stopAutoFlight; ++i)
    {
      flyToTarget(cruisePoints[i], speed, arriveThresh);
    }
  }
}

// D23飞行到目标点
void ControlModeManager::goToTargetPoint(float speed, float arriveThresh)
{
  flyToTarget(cruisePoints[0], speed, arriveThresh);
}

void ControlModeManager::processInstruction(const Json::Value &instructionData)
{
  string instructionID = instructionData["instructionID"].asString();
  InstructionNames instruction = instructions.at(instructionID);

  switch (instructionID)
  {
  case D1:
    USER_LOG_INFO("Processing D1: Takeoff");
    FlightTakeoffAndLanding(1);
    break;
  case D2:
    USER_LOG_INFO("Processing D2: Landing");
    FlightTakeoffAndLanding(2);
    break;
  case D3: // 前进
    USER_LOG_INFO("Processing D3: Move forward");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = 0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  case D4: // 后退
    USER_LOG_INFO("Processing D4: Move backward");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd1.x = -0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  case D5: // 向左
    USER_LOG_INFO("Processing D5: Move left");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.y = -0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  case D6: // 向右
    USER_LOG_INFO("Processing D6: Move right");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.y = 0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  case D7:
    switchControlMode(HangingMode);
    break;
  case D8: // 巡航
    USER_LOG_INFO("Processing D8: Cruise");
    float speed;
    float arriveThresh;

    cruisePoints = getWaypoints(instructionData["instructionParameter"]);
    speed = instructionData["instructionParameter"]["speed"].asFloat();
    arriveThresh = instructionData["instructionParameter"]["arriveThresh"].asFloat();

    cruisePath(speed, arriveThresh);
    break;
  case D9: // 设置前进速度
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
  case D10: // 设置前进加速度
    USER_LOG_INFO("Processing D10: Set acceleration");
    float acceleration = instructionData["instructionParameter"]['acceleration'].asFloat();
    float maxSpeed = instructionData["instructionParameter"]['maxSpeed'].asFloat();
    if (acceleration > 5.0f)
      acceleration = 5.0f;
    if (acceleration < 0.0f)
      acceleration = 0.0f; // 限制加速度范围

    setForwardAcceleration(acceleration, maxSpeed);
    break;
  case D11:
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
  case D13:
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
  case D21: // 上升
    USER_LOG_INFO("Processing D21: Ascend");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.z = 0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  case D22: // 下降
    USER_LOG_INFO("Processing D22: Descend");
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.z = -0.5f;
    FlightControl_VelocityControl(cmd);
    break;
  case D23: // 前往目标位置
    USER_LOG_INFO("Processing D23: Go to target point");
    float speed;
    float arriveThresh;

    cruisePoints = getWaypoints(instructionData["instructionParameter"]);
    speed = instructionData["instructionParameter"]["speed"].asFloat();
    arriveThresh = instructionData["instructionParameter"]["arriveThresh"].asFloat();

    goToTargetPoint();
    break;
  case D24: // 向左前方10°方向前进
    USER_LOG_INFO("Processing D24: Turn left");
    float angle = 10.0f * M_PI / 180.0f; // 弧度
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = 0.5 * cos(angle);
    cmd.y = -0.5 * sin(angle);
    FlightControl_VelocityControl(cmd);
    break;
  case D25: // 向右前方10°方向前进
    USER_LOG_INFO("Processing D23: Turn right");
    float angle = -10.0f * M_PI / 180.0f;
    T_DjiFlightControllerJoystickCommand cmd = {0};
    cmd.x = 0.5 * cos(angle);
    cmd.y = 0.5 * sin(angle);
    FlightControl_VelocityControl(cmd);
    break;
  case D26: // 返航
    USER_LOG_INFO("Processing D26: Return to home");
    T_DjiReturnCode returnCode = DjiFlightController_StartGoHome();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      USER_LOG_ERROR("Return to home failed, error code: 0x%08X", returnCode);
    }
    break;
  case D27:
  {
    FanController *fanController = FanController::get();
    int gear = instructionData["instructionParameter"].asInt();
    fanController->setGear(gear);
    if (controlMode != FanControllingMode)
      switchControlMode(FanControllingMode);
    break;
  }
  case D28:
  {
    LightController *lightController = LightController::get();
    int brightness = instructionData["instructionParameter"].asInt();
    lightController->setBrightness(brightness);
    if (controlMode != LightControllingMode)
      switchControlMode(LightControllingMode);
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
    T_DjiReturnCode returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
    }
    s_osalHandler->TaskSleepMs(1000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
    {
      USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
    }
  }
  case FanControllingMode:
  {
    FanController *fanController = FanController::get();
    fanController->setGear(0);
    break;
  }
  case LightControllingMode:
  {
    LightController *lightController = LightController::get();
    lightController->setBrightness(0);
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
