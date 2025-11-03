#ifndef NOTCH_TF_H
#define NOTCH_TF_H

typedef struct {
    // 系统状态变量
    double dOutPrev[2];  // 输出历史值
    double dInPrev[2];   // 输入历史值
    
    // 滤波器系数
    double dA0, dA1, dA2;   // 分母系数
    double dB0, dB1, dB2;   // 分子系数
    
    // 参数
    double dNotchFreq;      // 陷波频率
    double dNotchFreqPole;  // 极点频率
    double dNotchDampZero;  // 零点阻尼
    double dNotchDampPole;  // 极点阻尼
    double dTs;             // 采样时间
} SNotchTF;

// 函数声明
void NotchTFInit(SNotchTF* psFilter,double NotchFreq,double NotchFreqPole,double NotchDampZero,double samNotchDampPole,double sample_time);
double NotchTFUpdate(SNotchTF* psFilter, double dInput);
void NotchTFReset(SNotchTF* psFilter);

#endif 