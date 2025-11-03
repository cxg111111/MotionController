#include <stdio.h>
#include <math.h>
#include "Controler.h"
#include "Notch_TF.h"

/* 控制器参数定义 */
#define KP 500000.0
#define KI 10.0
#define KD 20.0
#define TS 0.001
#define LPF_FREQ 500.0
#define LPF_DAMP 0.8

#define NotchFreq 100.0
#define NotchFreqPole 100.0
#define NotchDampZero 0.01
#define NotchDampPole 0.05

#define SAMPLING_TIME 0.001 //采样时间 1ms

/* 初始化控制器 */
void ControllerInit(Controller *ctrl) {
    // 初始化PID控制器
    PIDControllerInit(&ctrl->pid, KP, KI, KD, SAMPLING_TIME);
    // 初始化低通滤波器
    LowPassFilterInit(&ctrl->lpf, LPF_FREQ, LPF_DAMP, SAMPLING_TIME);
    //初始化陷波滤波器
    NotchTFInit(&ctrl->notch, NotchFreq, NotchFreqPole, NotchDampZero, NotchDampPole, SAMPLING_TIME);
}

/* 控制器更新函数 */
double ControllerUpdate(Controller *ctrl, double error) {
    // 使用PID控制器计算控制输出
    double dPidOutput = PIDControllerUpdate(&ctrl->pid, error);
    // 使用低通滤波器对PID输出进行滤波
    double dFilteredOutput = LowPassFilterUpdate(&ctrl->lpf, dPidOutput);
    //使用陷波滤波器对输出进行滤波
    double dNotchOutput = NotchTFUpdate(&ctrl->notch, dFilteredOutput);
    
    return dNotchOutput;
}