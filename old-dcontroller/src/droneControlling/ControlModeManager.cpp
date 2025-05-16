#include <cassert>
#include <iostream>
#include "options.h"
#include "ControlModeManager.h"
#include "../gpioManipulators/FanController.h"
#include "../gpioManipulators/LightController.h"
#include <time.h>

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
    //cjwnote:此处添加D29指令，用于nano设备向无人机请求控制权
    instructions["D29"] = D29;
}

ControlModeManager::~ControlModeManager()
{
}

bool ControlModeManager::canProcessInstruction(const string& instructionID)
{
    return instructions.count(instructionID) > 0;
}

void ControlModeManager::emergencyStop()
{
    // switchControlMode(InstructionFollowingMode);
}

// 计算两点之间的球面距离（单位：米）
double ControlModeManager::calcDistance(const Waypoint& p1, const Waypoint& p2) {
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

//计算从p1指向p2的方位角（弧度，北为0，东为正pi/2）
double ControlModeManager::calcHeading(const Waypoint& p1, const Waypoint& p2) {
  double lat1 = p1.latitude * M_PI / 180.0;
  double lon1 = p1.longitude * M_PI / 180.0;
  double lat2 = p2.latitude * M_PI / 180.0;
  double lon2 = p2.longitude * M_PI / 180.0;
  double dlon = lon2 - lon1;
  double y = std::sin(dlon) * std::cos(lat2);
  double x = std::cos(lat1) * std::sin(lat2) -
             std::sin(lat1) * std::cos(lat2) * std::cos(dlon);
  return std::atan2(y, x);  // 范围[-pi, pi]
}

// 获取当前无人机位置（经纬度和高度）
Waypoint ControlModeManager::getCurrentPosition() {
  Waypoint pos;
  _FlightControllerGpsInfo gpsInfo;
  DjiFlightController_GetGpsInfo(&gpsInfo, 1000);
  pos.latitude = gpsInfo.latitude / 1e7; // 纬度
  pos.longitude = gpsInfo.longitude / 1e7; // 经度
  pos.altitude = gpsInfo.altitude; // 高度
  // pos.altitude = DjiUser_FlightControlGetValueOfRelativeHeight(); // 相对高度
  return pos;
}
/// 飞行到目标点，速度和到达阈值可选，默认速度2米每秒，到达阈值1米。到达阈值指的是与目标点的距离小于该值时，认为到达目标点。
void ControlModeManager::flyToTarget(const Waypoint& target, float speed = 2.0f, double arriveThresh = 1.0) {
  cruising = true;
  stopAutoFlight = false;

  while (!stopAutoFlight) {
      Waypoint curr = getCurrentPosition();
      double dist = calcDistance(curr, target);
      if (dist < arriveThresh) break;

      double heading = calcHeading(curr, target);
      double dAlt = target.altitude - curr.altitude;

      // 速度分量，x为北向，y为东向，z为上
      flyingCommand.x = speed * std::cos(heading); // 北向前进
      flyingCommand.y = speed * std::sin(heading); // 东向右进
      flyingCommand.z = dAlt;                      // 高度差，简单P，可加比例系数
      flyingCommand.yaw = 0.0f;                    // 不转向

      usleep(100 * 1000); // 100ms循环
  }
  // 到达或停止，全部通道清零悬停
  flyingCommand = {0};
  cruising = false;
}

//模拟操作员持续下发摇杆指令的线程函数
// 该函数会持续运行，直到程序结束或线程被终止
void* ControlModeManager::CommandSenderThread(void*) {
  static T_DjiFlightControllerJoystickMode lastMode = {};
  while (true) {
      if (needSetMode || memcmp(&lastMode, &currentJoystickMode, sizeof(currentJoystickMode)) != 0) {
          DjiFlightController_SetJoystickMode(currentJoystickMode);
          lastMode = currentJoystickMode;
          needSetMode = false;
      }
      DjiFlightController_ExecuteJoystickAction(flyingCommand);
      usleep(20000); // 20ms
  }
  return nullptr;
}

// D8巡航
void ControlModeManager::cruisePath() {
  if (cruisePointCount < 1) {
    USER_LOG_ERROR("No cruise points set.\r\n");
    return;
  } else {
    for (int i = 0; i < cruisePointCount && !stopAutoFlight; ++i){
      flyToTarget(cruisePoints[i]);
    }
  }
}

// D23飞行到目标点
void ControlModeManager::goToTargetPoint() {
  flyToTarget(targetPoint);
}

// 设置当前摇杆模式 speed:1, angle:2
void ControlModeManager::setCurrentJoystickMode(int mode) {
    switch (mode) {
        case 1:
            currentJoystickMode = {
                DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
                DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE
            };
            break;
        case 2:
            currentJoystickMode = {
                DJI_FLIGHT_CONTROLLER_HORIZONTAL_ANGLE_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
                DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE
            };
            break;
        default:
            break;
    }
    needSetMode = true;
}

void ControlModeManager::processInstruction(const Json::Value& instructionData)
{
    string instructionID = instructionData["instructionID"].asString();
    InstructionNames instruction = instructions.at(instructionID);

    switch (instruction){
      case D1:
        DjiFlightController_ObtainJoystickCtrlAuthority();
        DjiFlightController_StartTakeoff();
        break;

      case D2:
        DjiFlightController_StartLanding();
        break;

      case D3:   // 前进
        setCurrentJoystickMode(1); // 设置为速度控制模式
        flyingCommand = {0};
        flyingCommand.x =  0.5f;
        break;

      case D4:   // 后退
        setCurrentJoystickMode(1);
        flyingCommand = {0};
        flyingCommand.x =  -0.5f;
        break;

      case D5:   // 向左
        setCurrentJoystickMode(1);
        flyingCommand = {0};
        flyingCommand.y =  -0.5f;
        break;

      case D6:   // 向右
        setCurrentJoystickMode(1);
        flyingCommand = {0};
        flyingCommand.y =  0.5f;
        break;

      case D7:   // 悬停
        switchControlMode(HangingMode);
        break;
      case D8:   // 巡航
        //cjwnote:由于不清楚无人机3个巡航点如何设置，暂时不传入巡航点数据
        stopAutoFlight = true; // 终止上次巡航
        usleep(200*1000);
        pthread_t cruiseThread;
        pthread_create(&cruiseThread, nullptr, [](void*) -> void* {cruisePath(); return nullptr;}, nullptr);
        pthread_detach(cruiseThread);
        break;

      case D9:   // 设置飞行速度
        flySpeed = instructionData["instructionParameter"].asFloat();//假定速度由instructionData传入
        if(flySpeed > 5.0f) flySpeed = 5.0f;
        if(flySpeed < -5.0f) flySpeed = -5.0f; // 限制速度范围

        setCurrentJoystickMode(1);
        flyingCommand.x = flySpeed;
        break;

      case D10:  // 设置飞行加速度
        flyAcc = instructionData["instructionParameter"].asFloat();
        if(flyAcc > 0.5f) flyAcc = 0.5f; // 限制加速度范围
        float maxSpeed = 5.0f; // 限制最大速度

        int CONTROL_INTERVAL_MS = 33;
        float current_speed = 0.0;
        struct timespec start, end;
        clock_gettime(CLOCK_MONOTONIC, &start);

        while (current_speed < maxSpeed) {
            clock_gettime(CLOCK_MONOTONIC, &end);
            float elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
            current_speed += flyAcc * elapsed;

            if (current_speed > maxSpeed) {
              current_speed = maxSpeed;
              break;
            }
            setCurrentJoystickMode(1); // 设置为速度控制模式
            flyingCommand.x = current_speed;

            // 精确等待
            usleep(CONTROL_INTERVAL_MS * 1000);
        }
        setCurrentJoystickMode(1); // 设置为速度控制模式
        flyingCommand.x = flySpeed;
        break;

      case D11:  // 设置俯仰角度
        float pitchAngle = instructionData["instructionParameter"].asFloat();
        if (pitchAngle > 30.0f) pitchAngle = 30.0f;
        if (pitchAngle < -10.0f) pitchAngle = -10.0f; // 限制俯仰角范围

        setCurrentJoystickMode(2); // 设置为角度控制模式
        flyingCommand = {0};
        flyingCommand.x = pitchAngle;  // Pitch angle in degrees
        break;

      case D13:  // 设置航向角度
        float yawAngle = instructionData["instructionParameter"].asFloat();
        if (yawAngle > 180.0f) yawAngle = 180.0f;
        if (yawAngle < -180.0f) yawAngle = -180.0f; // 限制航向角范围
        
        setCurrentJoystickMode(2); // 设置为角度控制模式
        flyingCommand = {0};
        flyingCommand.yaw = yawAngle;  // Yaw angle in degrees
        break;

      case D21:  //上升
      setCurrentJoystickMode(1); // 设置为速度控制模式
        flyingCommand = {0};
        flyingCommand.z = 0.5f;
        break;

      case D22:  //下降
      setCurrentJoystickMode(1);
        flyingCommand = {0};
        flyingCommand.z = -0.5f;
        break;

      case D23:  //航向角向左转
        float angle = 10.0f * M_PI / 180.0f; // 弧度
        setCurrentJoystickMode(1);
        flyingCommand = {0};
        flyingCommand.x = 0.5 * cos(angle);
        flyingCommand.y = -0.5 * sin(angle);
        break;
        
      case D24:  //航向角向右转
        float angle = -10.0f * M_PI / 180.0f;
        setCurrentJoystickMode(1);
        flyingCommand = {0};
        flyingCommand.x = 0.5 * cos(angle);
        flyingCommand.y = 0.5 * sin(angle);
        break;

      case D25:  //前往目标位置
        //cjwnote:此处假定目标点数据由instructionData传入，包含经纬度和高度
        targetPoint.latitude = instructionData["instructionParameter"]["latitude"].asDouble();
        targetPoint.longitude = instructionData["instructionParameter"]["longitude"].asDouble();
        targetPoint.altitude = instructionData["instructionParameter"]["altitude"].asFloat();

        stopAutoFlight = true;
        usleep(200*1000);
        pthread_t targetThread;
        pthread_create(&targetThread, nullptr, [](void*) -> void* {goToTargetPoint(); return nullptr;}, nullptr);
        pthread_detach(targetThread);
        break;

      case D26:  //返航
        DjiFlightController_StartGoHome();
        break;

      case D27:{
        FanController* fanController = FanController::get();
        int gear = instructionData["instructionParameter"].asInt();
        fanController->setGear(gear);
        if (controlMode!= FanControllingMode)
            switchControlMode(FanControllingMode);
        break;
      }

    case D28:{
      LightController* lightController = LightController::get();
      int brightness = instructionData["instructionParameter"].asInt();
      lightController->setBrightness(brightness);
      if (controlMode!= LightControllingMode)
          switchControlMode(LightControllingMode);
      break;
    }

    case D29:{
      //cjw添加：用于nano设备向无人机请求控制权，通常情况下只需要启动时请求一次即可，若被遥控器接管后，再通过该命令请求接管
      T_DjiReturnCode ret = DjiFlightController_ObtainJoystickCtrlAuthority();
      if (ret != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
          // 获取出现错误，此处可以通过输出异常指令来提示用户
          USER_LOG_ERROR("Failed to obtain joystick control authority, error code: %d\r\n", ret);
          return;
      } else {
          // 成功获取控制权
          USER_LOG_INFO("Successfully obtained joystick control authority\r\n");
      }
      break;}
    }
}

void ControlModeManager::switchControlMode(ControlMode newMode)
{
    //For some modes, before leaving old mode, some cleanup-work
    //should be done.

    //cjwnote:此处将switch函数的controlMode改为newMode，保证此处第一次传入新状态时便能改变状态
    switch (newMode){
      case HangingMode:{
        stopAutoFlight = true; // 停止所有自动飞行
        setCurrentJoystickMode(1); // 设置为速度控制模式
        flyingCommand = {0};    // 所有轴都归零，自动悬停
        break;}
      case FanControllingMode:{
        FanController* fanController = FanController::get();
        fanController->setGear(0);
        break;}
      case LightControllingMode:{
        LightController* lightController = LightController::get();
        lightController->setBrightness(0);
        break;}
      case UnspecifiedMode:
        break;
    }

    controlMode = newMode;
}


void ControlModeManager::appendControllerStatus(Json::Value& status)
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

// runtime singleton
ControlModeManager* ControlModeManager::singleton = nullptr;

ControlModeManager* ControlModeManager::createInstance()
{
    assert(singleton == nullptr);
    singleton = new ControlModeManager();
    return singleton;
}

ControlModeManager* ControlModeManager::get()
{
    assert(singleton);
    return singleton;
}
