#include "fault_handler.h"

// ================== 全局变量定义 ==================
// 必须在 .c 文件中定义，与头文件中的 extern 声明对应
tAxisFaultCtx g_atAxisFaults[8];      // 8 个轴的故障上下文
tSystemFaultCtx g_tSystemFault;       // 系统级故障上下文

// ================== 函数实现 ==================

/**
 * @brief 初始化故障处理框架
 */
void vFault_Init(void) {
    uint8_t u8AxisIdx, u8FaultIdx;

    // 初始化所有轴
    for (u8AxisIdx = 0; u8AxisIdx < 8; u8AxisIdx++) {
        tAxisFaultCtx* ptCtx = &g_atAxisFaults[u8AxisIdx];

        for (u8FaultIdx = 0; u8FaultIdx < FAULT_MAX; u8FaultIdx++) {
            ptCtx->m_bSafini[u8FaultIdx] = false;
            ptCtx->m_bFmask[u8FaultIdx] = true;   // 默认监控
            ptCtx->m_bFdef[u8FaultIdx] = true;    // 默认响应
        }

        ptCtx->m_bInternalSafetyCond = true;
        ptCtx->m_bAxisFault = false;
    }

    // 初始化系统级安全
    g_tSystemFault.m_bSSafini = false;
    g_tSystemFault.m_bSFmask = true;
    g_tSystemFault.m_bSystemSafetyCond = true;
    g_tSystemFault.m_bSFault = false;
}

/**
 * @brief 更新单个轴的故障状态
 * @param u8AxisId 轴ID
 */
void vFault_UpdateAxis(uint8_t u8AxisId) {
    // 参数检查
    if (u8AxisId >= 8) {
        return; // 轴ID无效
    }

    tAxisFaultCtx* ptCtx = &g_atAxisFaults[u8AxisId];
    bool bTempFault = false;
    uint8_t u8FaultType;

    // 遍历所有故障类型
    for (u8FaultType = 0; u8FaultType < FAULT_MAX; u8FaultType++) {
        if (ptCtx->m_bRawFault[u8FaultType]) {
            // 应用 SAFINI 反相
            bool bSafiniInput = ptCtx->m_bSafini[u8FaultType] ? 
                                !ptCtx->m_bRawFault[u8FaultType] : 
                                ptCtx->m_bRawFault[u8FaultType];
            
            // XOR with internal safety condition
            bool bXorResult = bSafiniInput ^ ptCtx->m_bInternalSafetyCond;

            // AND with FMASK
            if (ptCtx->m_bFmask[u8FaultType]) {
                ptCtx->m_bFault[u8FaultType] = bXorResult;
                bTempFault |= ptCtx->m_bFault[u8FaultType];
            }
        }
    }

    ptCtx->m_bAxisFault = bTempFault;
}

/**
 * @brief 更新系统级故障状态
 */
void vFault_UpdateSystem(void) {
    bool bAnyAxisFault = false;
    uint8_t u8AxisIdx;

    // 检查是否有任意轴故障 (OR)
    for (u8AxisIdx = 0; u8AxisIdx < 8; u8AxisIdx++) {
        if (g_atAxisFaults[u8AxisIdx].m_bAxisFault) {
            bAnyAxisFault = true;
            break;
        }
    }

    // OR 轴故障与系统安全输入
    bool bOrResult = bAnyAxisFault || g_tSystemFault.m_bSystemSafetyCond;

    // 应用 S_SAFINI
    bool bSafiniInput = g_tSystemFault.m_bSSafini ? !bOrResult : bOrResult;

    // XOR with system safety condition
    bool bXorResult = bSafiniInput ^ g_tSystemFault.m_bSystemSafetyCond;

    // AND with S_FMASK
    bool bAndResult = bXorResult && g_tSystemFault.m_bSFmask;

    g_tSystemFault.m_bSFault = bAndResult;
}

/**
 * @brief 获取指定轴的故障状态
 * @param u8AxisId 轴ID
 * @return true 表示轴故障
 */
bool bFault_GetAxisFault(uint8_t u8AxisId) {
    if (u8AxisId >= 8) {
        return false; // 无效轴ID，返回无故障
    }
    return g_atAxisFaults[u8AxisId].m_bAxisFault;
}

/**
 * @brief 获取全局系统故障状态
 * @return true 表示系统故障
 */
bool bFault_GetSystemFault(void) {
    return g_tSystemFault.m_bSFault;
}