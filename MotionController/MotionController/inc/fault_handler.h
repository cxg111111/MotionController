#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

// ================== 枚举定义 ==================

/**
 * @brief 轴故障类型枚举
 */
typedef enum {
    FAULT_HARDWARE_RIGHT_LIMIT,
    FAULT_HARDWARE_LEFT_LIMIT,
    FAULT_NETWORK_ERROR,
    FAULT_MOTOR_OVERHEAT,
    FAULT_SOFTWARE_RIGHT_LIMIT,
    FAULT_SOFTWARE_LEFT_LIMIT,
    FAULT_ENCODER1_NOT_CONNECTED,
    FAULT_ENCODER2_NOT_CONNECTED,
    FAULT_DRIVE_FAULT,
    FAULT_ENCODER1_ERROR,
    FAULT_ENCODER2_ERROR,
    FAULT_NON_CRITICAL_POS_ERR,
    FAULT_CRITICAL_POS_ERR,
    FAULT_VELOCITY_LIMIT,
    FAULT_ACCELERATION_LIMIT,
    FAULT_OVERCURRENT,
    FAULT_SERVO_PROCESSOR_ALARM,
    FAULT_SAFE_TORQUE_OFF,
    FAULT_HSSI_NOT_CONNECTED,
    FAULT_HARDWARE_EMERGENCY_STOP,
    FAULT_MAX
} tFaultType; // 't' prefix for type

// ================== 结构体定义 ==================

/**
 * @brief 单个轴的故障配置与状态上下文
 */
typedef struct {
    // 配置参数
    bool m_bSafini[FAULT_MAX];        // SAFINI 反相位
    bool m_bFmask[FAULT_MAX];         // FMASK 监控位
    bool m_bFdef[FAULT_MAX];          // FDEF 响应位

    // 运行时状态
    bool m_bRawFault[FAULT_MAX];      // 原始故障信号
    bool m_bFault[FAULT_MAX];         // 处理后故障信号
    bool m_bAxisFault;                // 轴综合故障标志

    // 内部安全条件
    bool m_bInternalSafetyCond;       // 内部安全条件
} tAxisFaultCtx;

/**
 * @brief 系统级安全配置与状态上下文
 */
typedef struct {
    // 配置参数
    bool m_bSSafini;                  // 系统SAFINI
    bool m_bSFmask;                   // 系统FMASK

    // 运行时状态
    bool m_bSystemSafetyCond;         // 系统安全条件
    bool m_bSFault;                   // 全局系统故障标志
} tSystemFaultCtx;

// ================== 全局变量声明 ==================
// 使用 extern 声明，定义在 .c 文件中
extern tAxisFaultCtx g_atAxisFaults[8];   // g_: global, a: array
extern tSystemFaultCtx g_tSystemFault;    // g_: global

// ================== 函数声明 ==================

/**
 * @brief 初始化故障处理框架
 */
void vFault_Init(void);

/**
 * @brief 更新指定轴的故障状态
 * @param u8AxisId 轴ID (0~7)
 */
void vFault_UpdateAxis(uint8_t u8AxisId);

/**
 * @brief 更新系统级故障状态
 */
void vFault_UpdateSystem(void);

/**
 * @brief 获取指定轴的故障状态
 * @param u8AxisId 轴ID
 * @return true 表示轴处于故障状态
 */
bool bFault_GetAxisFault(uint8_t u8AxisId);

/**
 * @brief 获取全局系统故障状态
 * @return true 表示系统处于故障状态
 */
bool bFault_GetSystemFault(void);

#endif // FAULT_HANDLER_H