/**
 ********************************************************************
 * @file    evaluationV1.cpp
 * @brief   无人机移动指令安全性评估模块实现
 *
 * @copyright (c) 2024 Your Company. All rights reserved.
 *
 *********************************************************************
 该文件主要处理D3、D4、D5、D6、D21、D22指令的安全性评估；
 目前该文件的函数并没有集成到ControlModeManager中；
 使用方法为，在ControlModeManager中，调用MoveEvaluation_Init函数初始化评估上下文；
 调用MoveEvaluation_UpdateAvoidData函数更新避障数据；
 调用MoveEvaluation_ParseCommand函数解析指令；
 调用MoveEvaluation_EvaluateCommand函数进行评估；  

 */

/* Includes ------------------------------------------------------------------*/
#include "evaluationV1.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Private types -------------------------------------------------------------*/
typedef struct {
    T_DjiFcSubscriptionAvoidData avoidData;     // DJI避障数据
    bool isInitialized;                         // 是否已初始化
    bool hasValidData;                          // 是否有有效数据
} T_MoveEvaluationContext;

/* Private variables ---------------------------------------------------------*/
static T_MoveEvaluationContext g_moveEvalCtx = {0};

/* Private functions declaration ---------------------------------------------*/
static float CalculateSafeLength(float velocity);
static float GetDistanceByCommandType(E_CommandType commandType);
static bool GetSensorHealthByCommandType(E_CommandType commandType);
static const char* GetDirectionName(E_CommandType commandType);

/* Private functions definition ----------------------------------------------*/

/**
 * @brief 计算安全距离
 * @param velocity 移动速度（m/s）
 * @return 安全距离（m）
 */
static float CalculateSafeLength(float velocity)
{
    // safeLength = (replyTime + safeTime) * v + bufferLength
    return (REPLY_TIME_S + SAFE_TIME_S) * velocity + BUFFER_LENGTH_M;
}

/**
 * @brief 根据指令类型获取对应方向的距离
 * @param commandType 指令类型
 * @return 对应方向的距离
 */
static float GetDistanceByCommandType(E_CommandType commandType)
{
    switch (commandType) {
        case COMMAND_TYPE_D3:   // 前进
            return g_moveEvalCtx.avoidData.front;
        case COMMAND_TYPE_D4:   // 后退
            return g_moveEvalCtx.avoidData.back;
        case COMMAND_TYPE_D5:   // 向左
            return g_moveEvalCtx.avoidData.left;
        case COMMAND_TYPE_D6:   // 向右
            return g_moveEvalCtx.avoidData.right;
        case COMMAND_TYPE_D21:  // 上升
            return g_moveEvalCtx.avoidData.up;
        case COMMAND_TYPE_D22:  // 下降
            return g_moveEvalCtx.avoidData.down;
        default:
            return 100.0f; // 默认远距离
    }
}

/**
 * @brief 根据指令类型获取对应方向传感器健康状态
 * @param commandType 指令类型
 * @return 传感器是否健康
 */
static bool GetSensorHealthByCommandType(E_CommandType commandType)
{
    switch (commandType) {
        case COMMAND_TYPE_D3:   // 前进
            return g_moveEvalCtx.avoidData.frontHealth == 1;
        case COMMAND_TYPE_D4:   // 后退
            return g_moveEvalCtx.avoidData.backHealth == 1;
        case COMMAND_TYPE_D5:   // 向左
            return g_moveEvalCtx.avoidData.leftHealth == 1;
        case COMMAND_TYPE_D6:   // 向右
            return g_moveEvalCtx.avoidData.rightHealth == 1;
        case COMMAND_TYPE_D21:  // 上升
            return g_moveEvalCtx.avoidData.upHealth == 1;
        case COMMAND_TYPE_D22:  // 下降
            return g_moveEvalCtx.avoidData.downHealth == 1;
        default:
            return false;
    }
}

/**
 * @brief 根据指令类型获取方向名称
 * @param commandType 指令类型
 * @return 方向名称字符串
 */
static const char* GetDirectionName(E_CommandType commandType)
{
    switch (commandType) {
        case COMMAND_TYPE_D3:   return "前";
        case COMMAND_TYPE_D4:   return "后";
        case COMMAND_TYPE_D5:   return "左";
        case COMMAND_TYPE_D6:   return "右";
        case COMMAND_TYPE_D21:  return "上";
        case COMMAND_TYPE_D22:  return "下";
        default:                return "未知";
    }
}

/* Exported functions definition ---------------------------------------------*/

int MoveEvaluation_Init(void)
{
    if (g_moveEvalCtx.isInitialized) {
        return 0; // 已经初始化
    }

    // 初始化避障数据
    memset(&g_moveEvalCtx.avoidData, 0, sizeof(T_DjiFcSubscriptionAvoidData));
    
    // 设置默认值
    g_moveEvalCtx.avoidData.front = 100.0f;
    g_moveEvalCtx.avoidData.back = 100.0f;
    g_moveEvalCtx.avoidData.left = 100.0f;
    g_moveEvalCtx.avoidData.right = 100.0f;
    g_moveEvalCtx.avoidData.up = 100.0f;
    g_moveEvalCtx.avoidData.down = 100.0f;
    
    // 默认传感器未工作
    g_moveEvalCtx.avoidData.frontHealth = 0;
    g_moveEvalCtx.avoidData.backHealth = 0;
    g_moveEvalCtx.avoidData.leftHealth = 0;
    g_moveEvalCtx.avoidData.rightHealth = 0;
    g_moveEvalCtx.avoidData.upHealth = 0;
    g_moveEvalCtx.avoidData.downHealth = 0;
    
    g_moveEvalCtx.hasValidData = false;
    g_moveEvalCtx.isInitialized = true;

    return 0;
}

int MoveEvaluation_DeInit(void)
{
    if (!g_moveEvalCtx.isInitialized) {
        return -1;
    }

    memset(&g_moveEvalCtx, 0, sizeof(T_MoveEvaluationContext));
    return 0;
}

int MoveEvaluation_UpdateAvoidData(const T_DjiFcSubscriptionAvoidData *avoidData)
{
    if (!g_moveEvalCtx.isInitialized || avoidData == NULL) {
        return -1;
    }

    // 复制所有避障数据
    memcpy(&g_moveEvalCtx.avoidData, avoidData, sizeof(T_DjiFcSubscriptionAvoidData));
    g_moveEvalCtx.hasValidData = true;

    return 0;
}

int MoveEvaluation_ParseCommand(const char *commandStr, T_MoveCommand *command)
{
    if (!g_moveEvalCtx.isInitialized || commandStr == NULL || command == NULL) {
        return -1;
    }

    // 清空命令结构
    memset(command, 0, sizeof(T_MoveCommand));

    char cmdId[16];
    char params[64];
    
    // 分离指令ID和参数
    if (sscanf(commandStr, "%15[^:]:%63s", cmdId, params) == 2) {
        // 有参数的指令：D3:2.5
        command->velocity = atof(params);
    } else if (sscanf(commandStr, "%15s", cmdId) == 1) {
        // 无参数的指令：D3
        command->velocity = 0.5f; // 默认速度0.5m/s
        params[0] = '\0';
    } else {
        return -1; // 解析失败
    }

    // 根据指令ID设置指令类型
    if (strcmp(cmdId, "D3") == 0) {
        command->type = COMMAND_TYPE_D3;
    } else if (strcmp(cmdId, "D4") == 0) {
        command->type = COMMAND_TYPE_D4;
    } else if (strcmp(cmdId, "D5") == 0) {
        command->type = COMMAND_TYPE_D5;
    } else if (strcmp(cmdId, "D6") == 0) {
        command->type = COMMAND_TYPE_D6;
    } else if (strcmp(cmdId, "D21") == 0) {
        command->type = COMMAND_TYPE_D21;
    } else if (strcmp(cmdId, "D22") == 0) {
        command->type = COMMAND_TYPE_D22;
    } else {
        command->type = COMMAND_TYPE_UNKNOWN;
        return -1; // 不支持的指令
    }

    return 0;
}

int MoveEvaluation_EvaluateCommand(const T_MoveCommand *command, T_EvaluationDetails *details)
{
    if (!g_moveEvalCtx.isInitialized || command == NULL || details == NULL) {
        return -1;
    }

    // 初始化评估详情
    memset(details, 0, sizeof(T_EvaluationDetails));
    details->commandType = command->type;
    
    // 检查传感器状态和获取距离
    details->sensorHealthy = GetSensorHealthByCommandType(command->type);
    details->currentDistance = GetDistanceByCommandType(command->type);
    
    const char* directionName = GetDirectionName(command->type);
    
    // 如果传感器不健康或没有有效数据，返回不合理
    if (!details->sensorHealthy || !g_moveEvalCtx.hasValidData) {
        details->result = EVALUATION_RESULT_UNREASONABLE;
        snprintf(details->message, sizeof(details->message), 
                "指令不合理，%s向传感器故障或无数据", directionName);
        return 0;
    }
    
    // 计算所需安全距离
    details->requiredSafeLength = CalculateSafeLength(command->velocity);
    
    // 进行安全评估
    if (details->currentDistance > details->requiredSafeLength) {
        // 距离足够，指令合理
        details->result = EVALUATION_RESULT_REASONABLE;
        snprintf(details->message, sizeof(details->message), 
                "指令合理");
    } else {
        // 距离不够，指令不合理
        details->result = EVALUATION_RESULT_UNREASONABLE;
        snprintf(details->message, sizeof(details->message), 
                "指令不合理，距离太短");
    }

    return 0;
}

int MoveEvaluation_GetDirectionDistance(E_CommandType commandType, float *distance, bool *isHealthy)
{
    if (!g_moveEvalCtx.isInitialized || distance == NULL || isHealthy == NULL) {
        return -1;
    }

    *distance = GetDistanceByCommandType(commandType);
    *isHealthy = GetSensorHealthByCommandType(commandType);

    return 0;
}
