#pragma once
#include <string>
#include <jsoncpp/json/json.h>
#include "dji_flight_controller.h"
#include "dji_gimbal_manager.h"
#include <pthread.h>
#include <unistd.h>
#include <atomic>
#include <cmath>

using std::string;

enum ControlMode {
    UnspecifiedMode,
    HangingMode,
    FanControllingMode,
    LightControllingMode
};

class ControlModeManager{
public:
    bool canProcessInstruction(const string& instructionID);
    void processInstruction(const Json::Value& instruction);
    // void sendControlInstruction(T_DjiFlightControllerJoystickCommand& flyingCommand)

    void appendControllerStatus(Json::Value& status);

    ControlMode getControlMode() const;

    // Called when futher occllision may happen.
    void emergencyStop();
    // 持续下发摇杆命令的后台线程
    void* CommandSenderThread(void*);
    //设置当前摇杆模式
    void ControlModeManager::setCurrentJoystickMode(int mode);
    //工具函数
    double ControlModeManager::calcHeading(const Waypoint& p1, const Waypoint& p2);
    double ControlModeManager::calcDistance(const Waypoint& p1, const Waypoint& p2);
    //获取无人机当前GPS位置
    Waypoint ControlModeManager::getCurrentPosition();
    //向目标点飞行函数
    void ControlModeManager::flyToTarget(const Waypoint& target, float speed = 2.0f, double arriveThresh = 1.0);

private:
    void switchControlMode(ControlMode newMode);

private:
    ControlMode controlMode;

    enum InstructionNames{
        D1,  D2,  D3, D4, D5, D6, D7, D8,
        D9,  D10, D11, D13,
        D21, D22, D23, D24, D25, D26, D27, D28, D29
    };
    std::map<string, InstructionNames> instructions;

// runtime singleton
public:
    static ControlModeManager* createInstance();
    static ControlModeManager* get();
private:
    ControlModeManager();
    ~ControlModeManager();

    struct Waypoint {
        double latitude;
        double longitude;
        float altitude;
    };
    
    // 巡航点（最多三个）
    Waypoint cruisePoints[3];
    int cruisePointCount = 0;         // 当前巡航点数量
    
    // 目标点
    Waypoint targetPoint;

    T_DjiFlightControllerJoystickCommand flyingCommand = {0};      // 当前的摇杆指令
    T_DjiFlightControllerJoystickMode currentJoystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,    // 默认速度控制
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE
    };
    std::atomic<bool> needSetMode{true}; // 控制模式切换flag
    std::atomic<bool> cruising{false};           // 是否正在自动飞行
    std::atomic<bool> stopAutoFlight{false};     // 终止自动飞行

    float flySpeed    // 当前移动速度，D9时修改
    float flyAcc    // 当前加速度，D10时修改
    // 地球半径（米）
    constexpr double EARTH_RADIUS = 6371000.0;
private:
    static ControlModeManager* singleton;
};
