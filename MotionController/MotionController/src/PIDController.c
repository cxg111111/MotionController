#include "PIDController.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// PID控制器初始化
void PIDControllerInit(PIDController* pid, double kp, double ki, double kd, double sample_time) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sample_time = sample_time;
    
    // 初始化历史值
    pid->dFiInPrev[0] = 0.0;
    pid->dFiInPrev[1] = 0.0;
    pid->dFiOutPrev[0] = 0.0;
    pid->dFiOutPrev[1] = 0.0;
    
    pid->dFdInPrev[0] = 0.0;
    pid->dFdInPrev[1] = 0.0;
    pid->dFdOutPrev[0] = 0.0;
    pid->dFdOutPrev[1] = 0.0;
}

// PID控制器更新
double PIDControllerUpdate(PIDController* pid, double error) {
    volatile double fi_input, fd_input, fi_output, fd_output, pid_output;
    const double PI = 3.1415926;
    
    // 计算积分部分输入
    fi_input = error * pid->kp * pid->ki * (2.0 * PI) * (pid->sample_time / 2.0);
    
    // 计算微分部分输入
    fd_input = error * pid->kp * (1 / pid->kd) * (1.0 / 2.0 / PI) * (2.0 / pid->sample_time);
    
    // 积分部分计算
    fi_output = (1.0) * fi_input +
                (1.0) * pid->dFiInPrev[0] +
                (0.0) * pid->dFiInPrev[1] -
                (-1.0) * pid->dFiOutPrev[0] -
                (0.0) * pid->dFiOutPrev[1];
    
    // 更新积分部分历史值
    pid->dFiInPrev[1] = pid->dFiInPrev[0];
    pid->dFiInPrev[0] = fi_input;
    pid->dFiOutPrev[1] = pid->dFiOutPrev[0];
    pid->dFiOutPrev[0] = fi_output;
    
    // 微分部分计算
    fd_output = (1.0) * fd_input +
                (-1.0) * pid->dFdInPrev[0] +
                (0.0) * pid->dFdInPrev[1] -
                (1.0) * pid->dFdOutPrev[0] -
                (0.0) * pid->dFdOutPrev[1];
    
    // 更新微分部分历史值
    pid->dFdInPrev[1] = pid->dFdInPrev[0];
    pid->dFdInPrev[0] = fd_input;
    pid->dFdOutPrev[1] = pid->dFdOutPrev[0];
    pid->dFdOutPrev[0] = fd_output;
    
    // PID总输出
    pid_output = error * pid->kp + fi_output + fd_output;
    
    return pid_output;
}

// 重置PID控制器
void PIDControllerReset(PIDController* pid) {
    pid->dFiInPrev[0] = 0.0;
    pid->dFiInPrev[1] = 0.0;
    pid->dFiOutPrev[0] = 0.0;
    pid->dFiOutPrev[1] = 0.0;
    
    pid->dFdInPrev[0] = 0.0;
    pid->dFdInPrev[1] = 0.0;
    pid->dFdOutPrev[0] = 0.0;
    pid->dFdOutPrev[1] = 0.0;
}