/**
 ********************************************************************
 * @file    evaluationV1.h
 * @brief   无人机移动指令安全性评估模块
 *
 * @copyright (c) 2024 Your Company. All rights reserved.
 *
 *********************************************************************
 */

#ifndef EVALUATION_V1_H
#define EVALUATION_V1_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "dji_fc_subscription.h"  // 使用DJI的避障数据结构

/* Exported constants --------------------------------------------------------*/
#define SAFE_TIME_S         3.0f    // 安全时间（秒）
#define REPLY_TIME_S        3.0f    // 反应时间（秒）
#define BUFFER_LENGTH_M     5.0f    // 缓冲距离（米）

/* Exported types ------------------------------------------------------------*/

// 支持的指令类型
typedef enum {
    COMMAND_TYPE_D3 = 3,    // D3: 前进
    COMMAND_TYPE_D4 = 4,    // D4: 后退
    COMMAND_TYPE_D5 = 5,    // D5: 向左
    COMMAND_TYPE_D6 = 6,    // D6: 向右
    COMMAND_TYPE_D21 = 21,  // D21: 上升
    COMMAND_TYPE_D22 = 22,  // D22: 下降
    COMMAND_TYPE_UNKNOWN = 0
} E_CommandType;

// 评估结果
typedef enum {
    EVALUATION_RESULT_REASONABLE = 0,   // 指令合理
    EVALUATION_RESULT_UNREASONABLE      // 指令不合理
} E_EvaluationResult;

// 移动指令参数
typedef struct {
    E_CommandType type;                 // 指令类型
    float velocity;                     // 移动速度（m/s）
} T_MoveCommand;

// 评估详情
typedef struct {
    E_EvaluationResult result;          // 评估结果
    char message[128];                  // 结果消息
    float currentDistance;              // 当前对应方向距离
    float requiredSafeLength;           // 所需安全距离
    bool sensorHealthy;                 // 对应方向传感器是否正常
    E_CommandType commandType;          // 被评估的指令类型
} T_EvaluationDetails;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化移动指令评估模块
 * @return 0: 成功, -1: 失败
 */
int MoveEvaluation_Init(void);

/**
 * @brief 反初始化移动指令评估模块
 * @return 0: 成功, -1: 失败
 */
int MoveEvaluation_DeInit(void);

/**
 * @brief 更新障碍物距离信息（使用DJI避障数据）
 * @param avoidData DJI避障数据
 * @return 0: 成功, -1: 失败
 */
int MoveEvaluation_UpdateAvoidData(const T_DjiFcSubscriptionAvoidData *avoidData);

/**
 * @brief 解析移动指令字符串
 * @param commandStr 指令字符串（如"D3", "D4:1.5", "D21:0.8"等）
 * @param command 解析后的指令结构
 * @return 0: 成功, -1: 失败
 */
int MoveEvaluation_ParseCommand(const char *commandStr, T_MoveCommand *command);

/**
 * @brief 评估移动指令安全性
 * @param command 移动指令参数
 * @param details 评估详情（输出）
 * @return 0: 成功, -1: 失败
 */
int MoveEvaluation_EvaluateCommand(const T_MoveCommand *command, T_EvaluationDetails *details);

/**
 * @brief 获取指定方向的障碍物距离
 * @param commandType 指令类型（对应不同方向）
 * @param distance 对应方向距离（输出）
 * @param isHealthy 传感器是否正常（输出）
 * @return 0: 成功, -1: 失败
 */
int MoveEvaluation_GetDirectionDistance(E_CommandType commandType, float *distance, bool *isHealthy);

#ifdef __cplusplus
}
#endif

#endif // EVALUATION_V1_H
