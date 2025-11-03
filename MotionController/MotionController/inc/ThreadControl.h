#ifndef THREAD_CONTROL_H
#define THREAD_CONTROL_H
#include "Controlled_Device.h"
#include "Controler.h"
#include "FourthOrderTrajectoryPlanning.h"

#define SAMPLINGTIME 0.001     // 采样时间 1ms
#define TOTALSTEPS 1001         // 总步数
#define AXIS_COUNT 2  // 轴数为2

// 修改 ControlData 结构体为支持多轴
typedef struct {
    volatile double dTargetPosition[AXIS_COUNT];
    double dActualPosition[AXIS_COUNT];
    double dError[AXIS_COUNT];
    double dControlForce[AXIS_COUNT];
    double dOutputPosition[AXIS_COUNT];
} ControlData;
// 控制系统全局状态结构体

typedef struct {
    RigidBodyTF plant[AXIS_COUNT];
    Controller controller[AXIS_COUNT];
    ControlData ctrl_data;
    stTrajectoryPoint currentPoint[AXIS_COUNT];  // 每轴都有自己的轨迹
    stPlannerContext* pContext[AXIS_COUNT];
    int iControlStep;
    int bTrajectoryReady;
    int bControlRunning;
    FILE* pFile;
    // 添加以下两个成员用于支持独立轴控制
    int iControlStepPerAxis[AXIS_COUNT];  // 每个轴的独立步进计数器
    int bAxisActive[AXIS_COUNT];          // 每个轴的激活状态标记
} ControlSystemState;

// // 全局控制系统状态变量
// static ControlSystemState g_controlState = {
//     .pContext = { NULL, NULL },
//     .iControlStep = 0,
//     .bTrajectoryReady = 0,
//     .bControlRunning = 1,
//     .pFile = NULL
// };

// 执行计数器
static int g_executionCounter = 0;

// 全局控制变量
extern int g_bDataReceived;
extern struct RxData g_rxData;

// 函数声明
void* ControlThreadFunction(void* param);
void* SocketThreadFunction(void* param);
//csv文件处理线程函数
void* CSVWriterThreadFunction(void* param);

// 添加指令解析函数声明
void ProcessCommand(struct RxData* pRxData);

#endif