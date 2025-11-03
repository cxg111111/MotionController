#include "LowPassFilter.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.1415926

// 低通滤波器初始化
void LowPassFilterInit(LowPassFilter* lpf, double cutoff_freq, double damping, double sample_time) {
    lpf->sample_time = sample_time;
    
    // 计算滤波器系数
    double coef = 2.0 * PI * cutoff_freq * sample_time;
    double coef_sq = coef * coef;
    
    // 分子系数
    lpf->b0 = coef_sq;
    lpf->b1 = 2.0 * coef_sq;
    lpf->b2 = coef_sq;
    
    // 分母系数
    lpf->a0 = 4.0 + 4.0 * damping * coef + coef_sq;
    lpf->a1 = -8.0 + 2.0 * coef_sq;
    lpf->a2 = 4.0 - 4.0 * damping * coef + coef_sq;
    
    // 初始化历史值
    lpf->dInPrev[0] = 0.0;
    lpf->dInPrev[1] = 0.0;
    lpf->dOutPrev[0] = 0.0;
    lpf->dOutPrev[1] = 0.0;
}

// 低通滤波器更新
double LowPassFilterUpdate(LowPassFilter* lpf, double input) {
    double output;
    
    // 滤波器计算
    output = (lpf->b0 * input +
              lpf->b1 * lpf->dInPrev[0] +
              lpf->b2 * lpf->dInPrev[1] -
              lpf->a1 * lpf->dOutPrev[0] -
              lpf->a2 * lpf->dOutPrev[1]) / lpf->a0;
    
    // 更新历史值
    lpf->dInPrev[1] = lpf->dInPrev[0];
    lpf->dInPrev[0] = input;
    lpf->dOutPrev[1] = lpf->dOutPrev[0];
    lpf->dOutPrev[0] = output;
    
    return output;
}

// 重置低通滤波器
void LowPassFilterReset(LowPassFilter* lpf) {
    lpf->dInPrev[0] = 0.0;
    lpf->dInPrev[1] = 0.0;
    lpf->dOutPrev[0] = 0.0;
    lpf->dOutPrev[1] = 0.0;
}

// 获取滤波器输出
double LowPassFilterGetOutput(LowPassFilter* lpf) {
    return lpf->dOutPrev[0];
}