#ifndef CONTROLER_H
#define CONTROLER_H

#include "PIDController.h"
#include "LowPassFilter.h"
#include "Notch_TF.h"

// 控制器结构体
typedef struct {
    PIDController pid;      // PID控制器
    LowPassFilter lpf;      // 低通滤波器
    SNotchTF notch;         //陷波滤波器
} Controller;

// 函数声明
void ControllerInit(Controller *controller);
double ControllerUpdate(Controller * controller, double error);

#endif