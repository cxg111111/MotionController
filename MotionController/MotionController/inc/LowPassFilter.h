#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H

// 二阶低通滤波器结构体
typedef struct {
    // 滤波器系数
    double b0, b1, b2;  // 分子系数
    double a0, a1, a2;  // 分母系数
    
    double sample_time;  // 采样时间
    
    // 历史值
    double dInPrev[2];   // 输入历史值
    double dOutPrev[2];  // 输出历史值
} LowPassFilter;

// 函数声明
void LowPassFilterInit(LowPassFilter* lpf, double cutoff_freq, double damping, double sample_time);
double LowPassFilterUpdate(LowPassFilter* lpf, double input);
void LowPassFilterReset(LowPassFilter* lpf);
double LowPassFilterGetOutput(LowPassFilter* lpf);

#endif