#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

// PID控制器结构体;
typedef struct {
    // PID增益参数
    double kp;  // 比例增益
    double ki;  // 积分增益
    double kd;  // 微分增益
    double sample_time;  // 采样时间
    
    // 积分部分历史值
    double dFiInPrev[2];
    double dFiOutPrev[2];
    
    // 微分部分历史值
    double dFdInPrev[2];
    double dFdOutPrev[2];
} PIDController;

// 函数声明
void PIDControllerInit(PIDController* pid, double kp, double ki, double kd, double sample_time);
double PIDControllerUpdate(PIDController* pid, double error);
void PIDControllerReset(PIDController* pid);

#endif