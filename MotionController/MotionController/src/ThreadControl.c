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
    // 初始化整个结构体为0
    memset(&g_controlState, 0, sizeof(g_controlState));
    // 初始化特定成员
    g_controlState.bControlRunning = 1;
    for(int i = 0; i < AXIS_COUNT; i++) {
        g_controlState.pContext[i] = NULL;
    }
    
    // 初始化故障处理系统
    vFault_Init();
    errno_t err = fopen_s(&g_controlState.pFile, "control_data.csv", "w");
    if(err != 0)
    {
        fprintf(stderr,"Error: Cannot create CSV file!\n");
        return -1;
    }
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
            fprintf(stderr, "Trajectory planner initialization failed!\n");
            return -1;
        }
    }
    // 初始化每轴的独立状态
    for(int i = 0; i < AXIS_COUNT; i++) {
        g_controlState.iControlStepPerAxis[i] = 0;
        g_controlState.bAxisActive[i] = 0;  // 初始时不激活任何轴
    }

    g_controlState.bTrajectoryReady = 1;
    g_controlState.iControlStep = 0;
    g_controlState.bControlRunning = 1;
    printf("Control system initialized successfully for %d axes\n", AXIS_COUNT);
    return 0;
}

// 修改ExecuteControlStep函数以支持单轴和多轴控制
int ExecuteControlStep(int axisMask)
{
    if(!g_controlState.bTrajectoryReady || !g_controlState.bControlRunning)
        return -1;

    // 检查系统故障
    if (bFault_GetSystemFault()) {
        printf("SYSTEM FAULT DETECTED! Stopping control system.\n");
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
            printf("AXIS %d FAULT DETECTED! Switching to safe mode.\n", axis);
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
            printf("四阶轨迹调用结束");
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
    printf("Step: %d", g_controlState.iControlStep);
    for(int axis = 0; axis < AXIS_COUNT; axis++) {
        if (axisMask & (1 << axis)) {
            double time = (g_controlState.iControlStepPerAxis[axis] - 1) * SAMPLINGTIME; // 减1是因为上面已递增
            char mode_str = (SafetyData[axis].mode == CONTROL_MODE_CLOSED_LOOP) ? 'C' : 'O';
            printf(" | Axis%d: Time=%.3fs, Target=%.12f, Actual=%.15f, Error=%.13f, Force=%.9f (%c)", 
                   axis,
                   time,
                   g_controlState.ctrl_data.dTargetPosition[axis], 
                   g_controlState.ctrl_data.dActualPosition[axis], 
                   g_controlState.ctrl_data.dError[axis], 
                   g_controlState.ctrl_data.dControlForce[axis],
                   mode_str);
        }
    }
    printf("\n");

    // 写入CSV数据
    int controlMode[AXIS_COUNT];
    for(int axis = 0; axis < AXIS_COUNT; axis++) {
        controlMode[axis] = SafetyData[axis].mode;
    }
    
    // 使用第一个被控制轴的时间作为记录时间戳
    double recordTime = 0.0;
    for(int axis = 0; axis < AXIS_COUNT; axis++) {
        if (axisMask & (1 << axis)) {
            recordTime = (g_controlState.iControlStepPerAxis[axis] - 1) * SAMPLINGTIME;
            break;
        }
    }
    
    WriteCSVDataToBuffer(g_controlState.iControlStep, recordTime,
                        (double*)g_controlState.ctrl_data.dTargetPosition,
                        (double*)g_controlState.ctrl_data.dActualPosition,
                        (double*)g_controlState.ctrl_data.dError,
                        (double*)g_controlState.ctrl_data.dControlForce,
                        controlMode);
    
    // 数据有效性检查
    for(int axis = 0; axis < AXIS_COUNT; axis++) {
        // 只检查被控制的轴
        if (axisMask & (1 << axis)) {
            if (isnan(g_controlState.ctrl_data.dError[axis]) || isinf(g_controlState.ctrl_data.dError[axis]) ||
                isnan(g_controlState.ctrl_data.dControlForce[axis]) || isinf(g_controlState.ctrl_data.dControlForce[axis])) {
                printf("ERROR: Invalid numerical value detected for axis %d!\n", axis);
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
        return;
    }

    printf("Processing command: CMD=%d, Axis=%d\n", pRxData->iCMD, pRxData->axis);

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
                    printf("Controlling axis 0\n");
                } else if (pRxData->axis == 2) {
                    axisMask = 2;  // 控制轴1
                    printf("Controlling axis 1\n");
                } else if (pRxData->axis == 3) {
                    axisMask = 3;  // 控制轴0和轴1
                    printf("Controlling axis 0 and 1\n");
                } else {
                    printf("ERROR: Invalid axis value %d for CMD 1\n", pRxData->axis);
                    return;
                }
                
                // 执行控制步骤
                if(ExecuteControlStep(axisMask) != 0) {
                    printf("ERROR: Control step execution failed\n");
                }
            }
            break;
            
        case 2: // 重置控制步进计数器
            printf("Resetting control step counter\n");
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
                    printf("Controlling axis 0\n");
                } else if (pRxData->axis == 2) {
                    axisMask = 2;  // 控制轴1
                    printf("Controlling axis 1\n");
                } else if (pRxData->axis == 3) {
                    axisMask = 3;  // 控制轴0和轴1
                    printf("Controlling axis 0 and 1\n");
                } else {
                    printf("ERROR: Invalid axis value %d for CMD 3\n", pRxData->axis);
                    return;
                }
                
                int stepsToExecute = (int)pRxData->dParamData[0];
                printf("Executing %d control steps\n", stepsToExecute);
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
                        printf("One or more axes have reached maximum steps\n");
                        break;
                    }
                    
                    if(ExecuteControlStep(axisMask) != 0) {
                        printf("ERROR: Control step execution failed at step %d\n", i);
                        break;
                    }
                }
            }
            break;
            
        case 4: // 紧急停止
            printf("Emergency stop\n");
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
                printf("Axis %d switched to safe open-loop mode\n", axis);
            }                
            break;
            
        case 5: // 设置新的轨迹参数
            {
                int targetAxis = pRxData->axis;
                if (targetAxis < 0 || targetAxis >= AXIS_COUNT) {
                    printf("ERROR: Invalid axis number %d\n", targetAxis);
                    return;
                }
                
                printf("Setting new trajectory parameters for axis %d\n", targetAxis);
                
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
                    fprintf(stderr, "Trajectory planner initialization failed for axis %d!\n", targetAxis);
                } else {
                    printf("Trajectory planner reinitialized for axis %d\n", targetAxis);
                    printf("  Distance: %.6f\n", stInput.dDistance);
                    printf("  VMax: %.6f\n", stInput.dVMax);
                    printf("  AMax: %.6f\n", stInput.dAMax);
                    printf("  JMax: %.6f\n", stInput.dJMax);
                    printf("  DMax: %.6f\n", stInput.dDMax);
                }
            }
            break;
            
        case 6: // 修改控制器参数
            {
                int targetAxis = pRxData->axis;
                if (targetAxis < 0 || targetAxis >= AXIS_COUNT) {
                    printf("ERROR: Invalid axis number %d\n", targetAxis);
                    return;
                }
                
                printf("Modifying controller parameters for axis %d\n", targetAxis);
                
                // 修改PID控制器参数
                if (pRxData->dParamData[0] != 0.0) {
                    g_controlState.controller[targetAxis].pid.kp = pRxData->dParamData[0];
                    printf("Set Kp to %.6f\n", pRxData->dParamData[0]);
                }
                
                if (pRxData->dParamData[1] != 0.0) {
                    g_controlState.controller[targetAxis].pid.ki = pRxData->dParamData[1];
                    printf("Set Ki to %.6f\n", pRxData->dParamData[1]);
                }
                
                if (pRxData->dParamData[2] != 0.0) {
                    g_controlState.controller[targetAxis].pid.kd = pRxData->dParamData[2];
                    printf("Set Kd to %.6f\n", pRxData->dParamData[2]);
                }
                
                // // 修改低通滤波器参数
                // if (pRxData->dParamData[3] != 0.0) {
                //     g_controlState.controller[targetAxis].lpf.frequency = pRxData->dParamData[3];
                //     printf("Set LPF frequency to %.6f\n", pRxData->dParamData[3]);
                // }
            }
            break;
            
        case 7: // 查询系统状态
            {
                int targetAxis = pRxData->axis;
                if (targetAxis < 0 || targetAxis >= AXIS_COUNT) {
                    printf("ERROR: Invalid axis number %d\n", targetAxis);
                    return;
                }
                
                printf("System status for axis %d:\n", targetAxis);
                printf("  Control step: %d\n", g_controlState.iControlStep);
                printf("  Target position: %.12f\n", g_controlState.ctrl_data.dTargetPosition[targetAxis]);
                printf("  Actual position: %.15f\n", g_controlState.ctrl_data.dActualPosition[targetAxis]);
                printf("  Error: %.13f\n", g_controlState.ctrl_data.dError[targetAxis]);
                printf("  Control force: %.9f\n", g_controlState.ctrl_data.dControlForce[targetAxis]);
                printf("  Output position: %.12f\n", g_controlState.ctrl_data.dOutputPosition[targetAxis]);
                
                // 显示控制器参数
                printf("  Controller Kp: %.6f\n", g_controlState.controller[targetAxis].pid.kp);
                printf("  Controller Ki: %.6f\n", g_controlState.controller[targetAxis].pid.ki);
                printf("  Controller Kd: %.6f\n", g_controlState.controller[targetAxis].pid.kd);
                
                // 显示轨迹参数
                if (g_controlState.pContext[targetAxis] != NULL) {
                    printf("  Trajectory distance: %.6f\n", g_controlState.pContext[targetAxis]->stInput.dDistance);
                    printf("  Trajectory VMax: %.6f\n", g_controlState.pContext[targetAxis]->stInput.dVMax);
                    printf("  Trajectory AMax: %.6f\n", g_controlState.pContext[targetAxis]->stInput.dAMax);
                }
            }
            break;
            
        case 8: // 同时控制所有轴
            {
                // 控制所有轴 (使用全1掩码)
                int axisMask = (1 << AXIS_COUNT) - 1;
                printf("Controlling all axes\n");
                
                // 执行单步控制
                if(ExecuteControlStep(axisMask) != 0) {
                    printf("ERROR: Control step execution failed\n");
                }
            }
            break;
            
        case 9: // 执行所有轴的多步控制
            {
                // 控制所有轴 (使用全1掩码)
                int axisMask = (1 << AXIS_COUNT) - 1;
                int stepsToExecute = (int)pRxData->dParamData[0];
                printf("Executing %d control steps on all axes\n", stepsToExecute);
                for(int i = 0; i < stepsToExecute && g_controlState.iControlStep < TOTALSTEPS; i++) {
                    if(ExecuteControlStep(axisMask) != 0) {
                        printf("ERROR: Control step execution failed at step %d\n", i);
                        break;
                    }
                }
            }
            break;
            
        case 999: // 断开连接
            printf("Received disconnect command\n");
            g_controlState.bControlRunning = 0;
            break;
            
        default:
            printf("Unknown command: %d\n", pRxData->iCMD);
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
        printf("Socket data received in callback: CMD=%d\n", pData->iCMD);
    }
}

void* ControlThreadFunction(void* param)
{
    printf("Control thread started\n");
    printf("==========================================\n");
    printf("Waiting for socket commands to execute control steps...\n");
    
    // 初始化控制系统
    if(InitControlSystem() != 0)
    {
        fprintf(stderr, "Failed to initialize control system\n");
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
    
    printf("Control thread exiting\n");
    return NULL;
}

void* SocketThreadFunction(void* param)
{
    unsigned short port = *(unsigned short*)param;
    
    printf("Socket thread started on port %d\n", port);
    
    // 启动Socket服务器（会阻塞直到客户端断开）
    int result = RunSocketServer(port, SocketDataCallback);
    
    if(result == 0)
    {
        printf("Socket server completed normally\n");
    }
    else
    {
        fprintf(stderr, "Socket server encountered an error\n");
    }
    
    printf("Socket thread exiting\n");
    return NULL;
}