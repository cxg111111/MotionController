#ifndef SAFETY_FAULTS_H
#define SAFETY_FAULTS_H
#include "ThreadControl.h"

// 在全局变量区域添加以下定义
#define ERROR_THRESHOLD 0.0000000007      // 误差阈值 (单位: m)
#define AXIS_COUNT 2
// 安全控制模式枚举
typedef enum {
    CONTROL_MODE_CLOSED_LOOP = 0,  // 闭环控制模式
    CONTROL_MODE_OPEN_LOOP  = 1   // 开环控制模式
} ControlMode;

// 为每个轴添加安全控制状态
typedef struct {
    ControlMode mode;              // 当前控制模式
    double dLastValidOutput;     // 最后一次有效输出值
} SafetyControlData;

// 在全局变量区域添加安全控制数据
SafetyControlData SafetyData[AXIS_COUNT];

// 修改函数声明，使用不同的参数名避免冲突
double ApplySafetyControl(int axis, double control_force, double error, ControlSystemState* sysCtrlState);


#endif