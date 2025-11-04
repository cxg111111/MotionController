#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include "ThreadControl.h"
#include <stdio.h>
#include <windows.h>
#include <process.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "Socket.h"
#include "Safety_Faults.h"
#include "fault_handler.h"     // 故障处理头文件
#include "log.h"               // 添加日志头文件
// 控制器头文件
#include "Controler.h"          // 控制器
#include "Controlled_Device.h"  // 被控对象

static ControlSystemState g_controlState;

// 修改 InitControlData 函数
void InitControlData(ControlData* data) {
    for(int i = 0; i < AXIS_COUNT; i++) {
        data->dTargetPosition[i] = 0.0;
        data->dActualPosition[i] = 0.0;
        data->dError[i] = 0.0;
        data->dControlForce[i] = 0.0;
        data->dOutputPosition[i] = 0.0;

        // 初始化安全控制数据
        SafetyData[i].mode = CONTROL_MODE_CLOSED_LOOP;
        SafetyData[i].dLastValidOutput = 0.0;
    }
}

// 修改 InitControlSystem 函数中的初始化部分
int InitControlSystem(void)
{
    log_info("Initializing control system for %d axes", AXIS_COUNT);
    
    // 初始化整个结构体为0
    memset(&g_controlState, 0, sizeof(g_controlState));
    // 初始化特定成员
    g_controlState.bControlRunning = 1;
    for(int i = 0; i < AXIS_COUNT; i++) {
        g_controlState.pContext[i] = NULL;
    }
    
    // 初始化故障处理系统
    vFault_Init();
    log_debug("Fault handling system initialized");
    
    errno_t err = fopen_s(&g_controlState.pFile, "control_data.csv", "w");
    if(err != 0)
    {
        log_error("Cannot create CSV file!");
        return -1;
    }
    
    log_info("CSV file created successfully");
    
    //设置CSV文件句柄
    SetCSVFile(g_controlState.pFile);
    // 修改CSV文件头以适应多轴数据
    fprintf(g_controlState.pFile, "Step,Time(s)");
    fprintf(g_controlState.pFile, "TargetPosition_Axis,ActualPosition_Axis,Error_Axis,ControlForce_Axis,ControlMode_Axis");
    // 初始化每个轴的结构体,包括控制器，被控对象
    for(int i = 0; i < AXIS_COUNT; i++) {
        RigidBodyTFInit(&g_controlState.plant[i], 16.0, SAMPLINGTIME);
        ControllerInit(&g_controlState.controller[i]);
    }
    InitControlData(&g_controlState.ctrl_data);
    
    // 为每个轴规划轨迹
    stPlannerInput stInput;
    stInput.dDistance = 1.0;
    stInput.dVMax = 0.8;
    stInput.dAMax = 2.0;
    stInput.dJMax = 10.0;
    stInput.dDMax = 200.0;
    stInput.dSampleTime = 0.001;
    
    // 为每个轴进行轨迹规划
    for (int axis = 0; axis < AXIS_COUNT; axis++)
    {
        g_controlState.pContext[axis] = FourthOrderPlannerInit(&stInput);
        if (g_controlState.pContext[axis] == NULL) {
            //初始化失败，打印错误并退出
            log_error("Trajectory planner initialization failed!");
            return -1;
        }
    }
    
    log_debug("Trajectory planners initialized");
    
    // 初始化每轴的独立状态
    for(int i = 0; i < AXIS_COUNT; i++) {
        g_controlState.iControlStepPerAxis[i] = 0;
        g_controlState.bAxisActive[i] = 0;  // 初始时不激活任何轴
    }

    g_controlState.bTrajectoryReady = 1;
    g_controlState.iControlStep = 0;
    g_controlState.bControlRunning = 1;
    log_info("Control system initialized successfully for %d axes", AXIS_COUNT);
    return 0;
}


// 修改ExecuteControlStep函数以支持单轴和多轴控制
int ExecuteControlStep(int axisMask)
{
    if(!g_controlState.bTrajectoryReady || !g_controlState.bControlRunning) {
        log_warn("Control system not ready or not running");
        return -1;
    }

    // 检查系统故障
    if (bFault_GetSystemFault()) {
        log_error("SYSTEM FAULT DETECTED! Stopping control system.");
        g_controlState.bControlRunning = 0;
        return -1;
    }    
    
    // 为指定轴执行控制步骤
    for(int axis = 0; axis < AXIS_COUNT; axis++) {
        // 检查是否需要控制此轴 (axisMask的第axis位为1表示需要控制)
        if (!(axisMask & (1 << axis))) {
            continue; // 不需要控制的轴跳过
        }

        // 将轴设置为激活状态
        g_controlState.bAxisActive[axis] = 1;
        
        // 检查是否超过总步数
        if (g_controlState.iControlStepPerAxis[axis] >= TOTALSTEPS) {
            continue;
        }

        // 检查轴故障
        if (bFault_GetAxisFault(axis)) {
            log_warn("AXIS %d FAULT DETECTED! Switching to safe mode.", axis);
            SafetyData[axis].mode = CONTROL_MODE_OPEN_LOOP;
            g_controlState.ctrl_data.dControlForce[axis] = 0.0;
            continue;
        }

        // 计算该轴的时间
        double time = g_controlState.iControlStepPerAxis[axis] * SAMPLINGTIME;

        // 获取目标位置(使用预计算的轨迹)
        if (FourthOrderPlannerGetNextPoint(g_controlState.pContext[axis], &g_controlState.currentPoint[axis]) == 0)
        {
            g_controlState.ctrl_data.dTargetPosition[axis] = g_controlState.currentPoint[axis].dPos;
        }
        else
        {
            log_debug("四阶轨迹调用结束");
        }
        // 获取实际位置
        g_controlState.ctrl_data.dActualPosition[axis] = g_controlState.plant[axis].out_prev[0];

        // 计算误差
        g_controlState.ctrl_data.dError[axis] = g_controlState.ctrl_data.dTargetPosition[axis] - g_controlState.ctrl_data.dActualPosition[axis];
        
        // 更新控制器
        double raw_control_force = ControllerUpdate(&g_controlState.controller[axis], g_controlState.ctrl_data.dError[axis]);
        
        // 应用安全控制
        g_controlState.ctrl_data.dControlForce[axis] = ApplySafetyControl(axis, raw_control_force, g_controlState.ctrl_data.dError[axis], &g_controlState);
        
        // 更新故障检测系统状态
        vFault_UpdateAxis(axis);

        // 更新被控设备系统
        g_controlState.ctrl_data.dOutputPosition[axis] = RigidBodyTFUpdate(&g_controlState.plant[axis], g_controlState.ctrl_data.dControlForce[axis]);
        
        // 增加该轴的步进计数器
        g_controlState.iControlStepPerAxis[axis]++;
    }
    
    // 更新故障检测系统整体状态
    vFault_UpdateSystem();
    
    // 打印控制结果 - 只打印被控制的轴
    log_trace("Step: %d", g_controlState.iControlStep);
    for(int axis = 0; axis < AXIS_COUNT; axis++) {
        if (axisMask & (1 << axis)) {
            double time = (g_controlState.iControlStepPerAxis[axis] - 1) * SAMPLINGTIME; // 减1是因为上面已递增
            char mode_str = (SafetyData[axis].mode == CONTROL_MODE_CLOSED_LOOP) ? 'C' : 'O';
            log_trace("Axis%d: Time=%.3fs, Target=%.12f, Actual=%.15f, Error=%.13f, Force=%.9f (%c)", 
                   axis,
                   time,
                   g_controlState.ctrl_data.dTargetPosition[axis], 
                   g_controlState.ctrl_data.dActualPosition[axis], 
                   g_controlState.ctrl_data.dError[axis], 
                   g_controlState.ctrl_data.dControlForce[axis],
                   mode_str);
        }
    }

    // 数据有效性检查
    for(int axis = 0; axis < AXIS_COUNT; axis++) {
        // 只检查被控制的轴
        if (axisMask & (1 << axis)) {
            if (isnan(g_controlState.ctrl_data.dError[axis]) || isinf(g_controlState.ctrl_data.dError[axis]) ||
                isnan(g_controlState.ctrl_data.dControlForce[axis]) || isinf(g_controlState.ctrl_data.dControlForce[axis])) {
                log_error("Invalid numerical value detected for axis %d!", axis);
                return -1;
            }
        }
    }
    
    g_controlState.iControlStep++;
    return 0;
}
// 修改ProcessCommand函数以支持单轴和多轴控制
void ProcessCommand(struct RxData* pRxData) {
    if (pRxData == NULL) {
        log_warn("Received null command data");
        return;
    }

    log_debug("Processing command: CMD=%d, Axis=%d", pRxData->iCMD, pRxData->axis);

    switch (pRxData->iCMD) {
        case 1: // 控制轴运动
            {
                int axisMask = 0;
                // 根据axis值确定控制哪些轴
                // axis=1 控制轴0
                // axis=2 控制轴1
                // axis=3 控制轴0和轴1
                if (pRxData->axis == 1) {
                    axisMask = 1;  // 控制轴0
                    log_info("Controlling axis 0");
                } else if (pRxData->axis == 2) {
                    axisMask = 2;  // 控制轴1
                    log_info("Controlling axis 1");
                } else if (pRxData->axis == 3) {
                    axisMask = 3;  // 控制轴0和轴1
                    log_info("Controlling axis 0 and 1");
                } else {
                    log_error("Invalid axis value %d for CMD 1", pRxData->axis);
                    return;
                }
                
                // 执行控制步骤
                if(ExecuteControlStep(axisMask) != 0) {
                    log_error("Control step execution failed");
                }
            }
            break;
            
        case 2: // 重置控制步进计数器
            log_info("Resetting control step counter");
            g_controlState.iControlStep = 0;
            // 同时重置各轴的步进计数器
            for(int i = 0; i < AXIS_COUNT; i++) {
                g_controlState.iControlStepPerAxis[i] = 0;
                g_controlState.bAxisActive[i] = 0;
            }
            break;
            
        case 3: // 执行多步控制
            {
                int axisMask = 0;
                // 根据axis值确定控制哪些轴
                if (pRxData->axis == 1) {
                    axisMask = 1;  // 控制轴0
                    log_info("Controlling axis 0");
                } else if (pRxData->axis == 2) {
                    axisMask = 2;  // 控制轴1
                    log_info("Controlling axis 1");
                } else if (pRxData->axis == 3) {
                    axisMask = 3;  // 控制轴0和轴1
                    log_info("Controlling axis 0 and 1");
                } else {
                    log_error("Invalid axis value %d for CMD 3", pRxData->axis);
                    return;
                }
                
                int stepsToExecute = (int)pRxData->dParamData[0];
                log_info("Executing %d control steps", stepsToExecute);
                for(int i = 0; i < stepsToExecute; i++) {
                    // 检查所有涉及轴是否都未超过总步数
                    int canContinue = 1;
                    for(int axis = 0; axis < AXIS_COUNT; axis++) {
                        if ((axisMask & (1 << axis)) && g_controlState.iControlStepPerAxis[axis] >= TOTALSTEPS) {
                            canContinue = 0;
                            break;
                        }
                    }
                    
                    if (!canContinue) {
                        log_info("One or more axes have reached maximum steps");
                        break;
                    }
                    
                    if(ExecuteControlStep(axisMask) != 0) {
                        log_error("Control step execution failed at step %d", i);
                        break;
                    }
                }
            }
            break;
            
        case 4: // 紧急停止
            log_warn("Emergency stop triggered");
            g_controlState.bControlRunning = 0;
            // 触发硬件紧急停止故障
            for(int axis = 0; axis < AXIS_COUNT && axis < 8; axis++) {
                g_atAxisFaults[axis].m_bRawFault[FAULT_HARDWARE_EMERGENCY_STOP] = true;
                vFault_UpdateAxis(axis);
            }
            vFault_UpdateSystem();
            // 切换到开环模式作为安全措施
            for(int axis = 0; axis < AXIS_COUNT; axis++) {
                SafetyData[axis].mode = CONTROL_MODE_OPEN_LOOP;
                g_controlState.ctrl_data.dControlForce[axis] = 0.0; // 清除控制力
                log_info("Axis %d switched to safe open-loop mode", axis);
            }                
            break;
            
        case 5: // 设置新的轨迹参数
            {
                int targetAxis = pRxData->axis;
                if (targetAxis < 0 || targetAxis >= AXIS_COUNT) {
                    log_error("Invalid axis number %d", targetAxis);
                    return;
                }
                
                log_info("Setting new trajectory parameters for axis %d", targetAxis);
                
                // 重新初始化轨迹规划器
                stPlannerInput stInput;
                stInput.dDistance = (pRxData->dParamData[0] != 0.0) ? pRxData->dParamData[0] : 1.0;
                stInput.dVMax = (pRxData->dParamData[1] != 0.0) ? pRxData->dParamData[1] : 0.8;
                stInput.dAMax = (pRxData->dParamData[2] != 0.0) ? pRxData->dParamData[2] : 2.0;
                stInput.dJMax = (pRxData->dParamData[3] != 0.0) ? pRxData->dParamData[3] : 10.0;
                stInput.dDMax = (pRxData->dParamData[4] != 0.0) ? pRxData->dParamData[4] : 200.0;
                stInput.dSampleTime = SAMPLINGTIME;
                
                // 释放旧的轨迹规划器
                if (g_controlState.pContext[targetAxis] != NULL) {
                    free(g_controlState.pContext[targetAxis]);
                    g_controlState.pContext[targetAxis] = NULL;
                }
                
                // 创建新的轨迹规划器
                g_controlState.pContext[targetAxis] = FourthOrderPlannerInit(&stInput);
                if (g_controlState.pContext[targetAxis] == NULL) {
                    log_error("Trajectory planner initialization failed for axis %d!", targetAxis);
                } else {
                    log_info("Trajectory planner reinitialized for axis %d", targetAxis);
                    log_debug("Distance: %.6f, VMax: %.6f, AMax: %.6f, JMax: %.6f, DMax: %.6f",
                             stInput.dDistance, stInput.dVMax, stInput.dAMax, stInput.dJMax, stInput.dDMax);
                }
            }
            break;
            
        case 6: // 修改控制器参数
            {
                int targetAxis = pRxData->axis;
                if (targetAxis < 0 || targetAxis >= AXIS_COUNT) {
                    log_error("Invalid axis number %d", targetAxis);
                    return;
                }
                
                log_info("Modifying controller parameters for axis %d", targetAxis);
                
                // 修改PID控制器参数
                if (pRxData->dParamData[0] != 0.0) {
                    g_controlState.controller[targetAxis].pid.kp = pRxData->dParamData[0];
                    log_info("Set Kp to %.6f", pRxData->dParamData[0]);
                }
                
                if (pRxData->dParamData[1] != 0.0) {
                    g_controlState.controller[targetAxis].pid.ki = pRxData->dParamData[1];
                    log_info("Set Ki to %.6f", pRxData->dParamData[1]);
                }
                
                if (pRxData->dParamData[2] != 0.0) {
                    g_controlState.controller[targetAxis].pid.kd = pRxData->dParamData[2];
                    log_info("Set Kd to %.6f", pRxData->dParamData[2]);
                }
            }
            break;
            
        case 7: // 查询系统状态
            {
                int targetAxis = pRxData->axis;
                if (targetAxis < 0 || targetAxis >= AXIS_COUNT) {
                    log_error("Invalid axis number %d", targetAxis);
                    return;
                }
                
                log_info("System status for axis %d:", targetAxis);
                log_info("Control step: %d", g_controlState.iControlStep);
                log_debug("Target position: %.12f", g_controlState.ctrl_data.dTargetPosition[targetAxis]);
                log_debug("Actual position: %.15f", g_controlState.ctrl_data.dActualPosition[targetAxis]);
                log_debug("Error: %.13f", g_controlState.ctrl_data.dError[targetAxis]);
                log_debug("Control force: %.9f", g_controlState.ctrl_data.dControlForce[targetAxis]);
                log_debug("Output position: %.12f", g_controlState.ctrl_data.dOutputPosition[targetAxis]);
                
                // 显示控制器参数
                log_debug("Controller Kp: %.6f", g_controlState.controller[targetAxis].pid.kp);
                log_debug("Controller Ki: %.6f", g_controlState.controller[targetAxis].pid.ki);
                log_debug("Controller Kd: %.6f", g_controlState.controller[targetAxis].pid.kd);
                
                // 显示轨迹参数
                if (g_controlState.pContext[targetAxis] != NULL) {
                    log_debug("Trajectory distance: %.6f", g_controlState.pContext[targetAxis]->stInput.dDistance);
                    log_debug("Trajectory VMax: %.6f", g_controlState.pContext[targetAxis]->stInput.dVMax);
                    log_debug("Trajectory AMax: %.6f", g_controlState.pContext[targetAxis]->stInput.dAMax);
                }
            }
            break;
            
        case 8: // 同时控制所有轴
            {
                // 控制所有轴 (使用全1掩码)
                int axisMask = (1 << AXIS_COUNT) - 1;
                log_info("Controlling all axes");
                
                // 执行单步控制
                if(ExecuteControlStep(axisMask) != 0) {
                    log_error("Control step execution failed");
                }
            }
            break;
            
        case 9: // 执行所有轴的多步控制
            {
                // 控制所有轴 (使用全1掩码)
                int axisMask = (1 << AXIS_COUNT) - 1;
                int stepsToExecute = (int)pRxData->dParamData[0];
                log_info("Executing %d control steps on all axes", stepsToExecute);
                for(int i = 0; i < stepsToExecute && g_controlState.iControlStep < TOTALSTEPS; i++) {
                    if(ExecuteControlStep(axisMask) != 0) {
                        log_error("Control step execution failed at step %d", i);
                        break;
                    }
                }
            }
            break;
            
        case 999: // 断开连接
            log_info("Received disconnect command");
            g_controlState.bControlRunning = 0;
            break;
            
        default:
            log_warn("Unknown command: %d", pRxData->iCMD);
            break;
    }
}


// 在 ExecuteSocketCommand 函数中添加对轴参数的支持
void ExecuteSocketCommand(void)
{
    if(g_bDataReceived)
    {
        // 使用新的指令解析函数处理命令
        ProcessCommand(&g_rxData);
        
        g_bDataReceived = 0;
    }
}

// 清理控制资源
void CleanupControlSystem(void)
{
    if(g_controlState.pFile != NULL)
    {
        fclose(g_controlState.pFile);
        g_controlState.pFile = NULL;
    }
    g_controlState.bTrajectoryReady = 0;
    g_controlState.bControlRunning = 0;
    printf("Control system cleaned up\n");
}
// Socket回调函数
void SocketDataCallback(struct RxData* pData)
{
    // 数据已经在全局变量g_rxData中，这里可以添加额外处理
    if(pData != NULL)
    {
        log_debug("Socket data received in callback: CMD=%d", pData->iCMD);
    }
}

void* ControlThreadFunction(void* param)
{
    log_info("Control thread started");
    log_info("==========================================");
    log_info("Waiting for socket commands to execute control steps...");
    
    // 初始化控制系统
    if(InitControlSystem() != 0)
    {
        log_error("Failed to initialize control system");
        return NULL;
    }
    
    // 控制线程的主循环
    while(g_controlState.bControlRunning)
    {
        // 检查并处理Socket命令
        ExecuteSocketCommand();
        
        // 短暂延时避免CPU占用过高
        Sleep(10);
    }
    
    // 清理资源
    CleanupControlSystem();
    
    log_info("Control thread exiting");
    return NULL;
}

void* SocketThreadFunction(void* param)
{
    unsigned short port = *(unsigned short*)param;
    
    log_info("Socket thread started on port %d", port);
    
    // 启动Socket服务器（会阻塞直到客户端断开）
    int result = RunSocketServer(port, SocketDataCallback);
    
    if(result == 0)
    {
        log_info("Socket server completed normally");
    }
    else
    {
        log_error("Socket server encountered an error");
    }
    
    log_info("Socket thread exiting");
    return NULL;
}