#pragma once
#include <string>
#include <jsoncpp/json/json.h>
#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_typedef.h"
#include <cmath>
#include <thread>
#include <atomic>
#include <mutex>

using std::string;

enum ControlMode {
    UnspecifiedMode,
    HangingMode
};

struct Waypoint {
    double latitude;
    double longitude;
    float altitude;
};

class ControlModeManager{
public:
    T_DjiReturnCode initialize();
    T_DjiReturnCode FlightControlDeInit();

    bool canProcessInstruction(const string& instructionID);
    void processInstruction(const Json::Value& instruction);

    void appendControllerStatus(Json::Value& status);

    ControlMode getControlMode() const;

    // Called when futher occllision may happen.
    // void emergencyStop();
    
    // 飞行控制相关函数
    void FlightControl_VelocityControl(T_DjiFlightControllerJoystickCommand command);
    void FlightControl_setPitchAndYaw(T_DjiFlightControllerJoystickCommand command);
    void setForwardAcceleration(float acceleration, float maxSpeed = 5.0f, uint32_t durationMs = 10*1000);
    // 紧急制动相关函数
    void emergencyBrake();
    void hover();
    // 路径点相关函数
    double calcDistance(const Waypoint &p1, const Waypoint &p2);
    double calcHeading(const Waypoint &p1, const Waypoint &p2);
    // 巡航路径相关函数
    void flyToTarget(const Waypoint &target, float speed = 2.0f, float arriveThresh = 1.0f);
    Waypoint getCurrentPosition();
    void cruisePath(float speed, float arriveThresh);
    void goToTargetPoint(const Waypoint& target, float speed = 2.0f, float arriveThresh = 1.0f);

    // 设置路径点相关函数
    void saveWaypoint(const Waypoint& waypoint);
    void setHomePoint(const Waypoint& waypoint);
    void clearAllWaypoints();
    const std::vector<Waypoint>& getSavedWaypoints() const;
    const Waypoint& getHomePoint() const;

private:
    void switchControlMode(ControlMode newMode);

    bool FlightTakeoffAndLanding(int ID);

private:
    ControlMode controlMode;

    enum InstructionNames{
        D1,  D2,  D3, D4, D5, D6, D7, D8,
        D9,  D10, D11, D13,
        D21, D22, D23, D24, D25, D26, D27, D28,
        D29, D30, D31  // D29: 保存路径点, D30: 设置返航点, D31: 清除所有路径点
    };
    std::map<string, InstructionNames> instructions;

    static T_DjiOsalHandler *s_osalHandler;

    // 巡航点和返航点
    std::vector<Waypoint> cruisePoints;  // 用于存储巡航点和保存的路径点
    Waypoint homePoint;                  // 返航点
    static constexpr double EARTH_RADIUS = 6371000.0; // 地球半径，单位：米

// runtime singleton
public:
    static ControlModeManager* createInstance();
    static ControlModeManager* get();
private:
    ControlModeManager();
    ~ControlModeManager();

private:
    static ControlModeManager* singleton;

    bool MotorStartedCheck();
    bool TakeOffInAirCheck();
    bool TakeoffFinishedCheck();
    
    // 检查降落是否完成
    bool LandFinishedCheck();
    
    // 检查动作是否开始
    bool CheckActionStarted(E_DjiFcSubscriptionDisplayMode mode);

    std::thread controlThread;
    std::atomic<bool> isControlling;  // 使用原子变量确保线程安全
    T_DjiFlightControllerJoystickCommand currentCommand;  // 这个变量在控制循环中只读，不需要原子性
    std::mutex controlMutex;

    // 新增成员变量
    struct {
        float acceleration = 0.0f;
        float maxSpeed = 5.0f;
        uint32_t durationMs = 0;
        uint32_t elapsedMs = 0;
        bool isAccelerating = false;
    } accelerationControl;

    struct {
        Waypoint target;
        float speed = 2.0f;
        float arriveThresh = 1.0f;
        bool isFlying = false;
    } positionControl;

    void controlLoop();
    void stopControl();
    void accelerationControlLoop();
    void positionControlLoop();
};
