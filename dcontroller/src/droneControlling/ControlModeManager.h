#pragma once
#include <string>
#include <jsoncpp/json/json.h>
#include "dji_flight_controller.h"
#include "dji_fc_subscription.h"
#include "dji_logger.h"
#include "dji_platform.h"
#include "dji_typedef.h"
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

    void appendControllerStatus(Json::Value& status);

    ControlMode getControlMode() const;

    // Called when futher occllision may happen.
    void emergencyStop();

private:
    void switchControlMode(ControlMode newMode);

    T_DjiReturnCode initialize();
    T_DjiReturnCode FlightControlDeInit(void);
private:
    ControlMode controlMode;

    enum InstructionNames{
        D1,  D2,  D3, D4, D5, D6, D7, D8,
        D9,  D10, D11, D13,
        D21, D22, D23, D24, D25, D26, D27, D28
    };
    std::map<string, InstructionNames> instructions;

    static T_DjiOsalHandler *s_osalHandler = NULL;

    struct Waypoint {
        double latitude;
        double longitude;
        float altitude;
    };

    // 巡航点（最多三个）
    Waypoint cruisePoints[3];
    int cruisePointCount = 0;         // 当前巡航点数量

    constexpr double EARTH_RADIUS = 6371000.0; // 地球半径，单位：米
// runtime singleton
public:
    static ControlModeManager* createInstance();
    static ControlModeManager* get();
private:
    ControlModeManager();
    ~ControlModeManager();

private:
    static ControlModeManager* singleton;
};
