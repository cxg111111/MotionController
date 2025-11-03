#include "Safety_Faults.h"
#include "stdbool.h"
#include <stdio.h>
#include "math.h"
#include "ThreadControl.h"
#include "fault_handler.h"  // 添加fault_handler头文件

// 添加一个新的函数用于处理安全控制逻辑
double ApplySafetyControl(int axis, double control_force, double error, ControlSystemState* sysCtrlState) {
    // 检查是否进入安全模式
    for(int axis = 0; axis < AXIS_COUNT; axis++) {
        if(sysCtrlState->iControlStep * SAMPLINGTIME < sysCtrlState->pContext[axis]->dTa) {
        
            if (fabs(error) > ERROR_THRESHOLD && SafetyData[axis].mode == CONTROL_MODE_CLOSED_LOOP) {
                printf("WARNING: Error threshold exceeded on axis %d (Error: %.13f > %.13f)\n", 
                    axis, fabs(error), ERROR_THRESHOLD);
                printf("Switching to open-loop control for axis %d\n", axis);
        
                SafetyData[axis].mode = CONTROL_MODE_OPEN_LOOP;
                SafetyData[axis].dLastValidOutput = control_force;
        
                // 设置fault_handler中的相应故障标志
                if (axis < 8) {  // 确保轴ID在有效范围内
                    // 将非关键位置误差设置为故障
                    g_atAxisFaults[axis].m_bRawFault[FAULT_NON_CRITICAL_POS_ERR] = true;
                    vFault_UpdateAxis(axis);
                    vFault_UpdateSystem();
                }
                // 在进入安全模式时可以选择输出零力或保持最后一次有效输出
                return 0.0; // 输出零力作为安全措施
            }
        }
    }
       
    // 正常闭环控制模式
    SafetyData[axis].dLastValidOutput = control_force;
    return control_force;
}