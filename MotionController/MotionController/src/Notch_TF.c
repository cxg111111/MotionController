#include "Notch_TF.h"
#include <math.h>

#define CONST_PI 3.14159265358979323846

void NotchTFInit(SNotchTF* psFilter, double NotchFreq, double NotchFreqPole, double NotchDampZero,double samNotchDampPole, double sample_time)
{
    // 初始化参数
    psFilter->dNotchFreq = NotchFreq;
    psFilter->dNotchFreqPole = NotchFreqPole;
    psFilter->dNotchDampZero = NotchDampZero;
    psFilter->dNotchDampPole = samNotchDampPole;
    psFilter->dTs = sample_time; // [s]
    
    // 初始化历史值
    psFilter->dOutPrev[0] = 0.0;
    psFilter->dOutPrev[1] = 0.0;
    psFilter->dInPrev[0] = 0.0;
    psFilter->dInPrev[1] = 0.0;
    
    // 模拟参数初始化
    double dT = psFilter->dTs;
    double dFz = psFilter->dNotchFreq;
    double dFp = psFilter->dNotchFreqPole;
    double dDz = psFilter->dNotchDampZero;
    double dDp = psFilter->dNotchDampPole;
    
    // 离散化系数计算
    double dOmegaZ = 2.0 * CONST_PI * dFz; // 零点离散化系数
    double dOmegaP = 2.0 * CONST_PI * dFp; // 极点离散化系数
    
    // 分子系数计算
    psFilter->dB0 = 1.0 + 2.0 * dDz / dOmegaZ / dT + 4.0 / (dOmegaZ * dOmegaZ * dT * dT);
    psFilter->dB1 = 2.0 - 8.0 / (dOmegaZ * dOmegaZ * dT * dT);
    psFilter->dB2 = 1.0 - 2.0 * dDz / dOmegaZ / dT + 4.0 / (dOmegaZ * dOmegaZ * dT * dT);
    
    // 分母系数计算
    psFilter->dA0 = 1.0 + 2.0 * dDp / dOmegaP / dT + 4.0 / (dOmegaP * dOmegaP * dT * dT);
    psFilter->dA1 = 2.0 - 8.0 / (dOmegaP * dOmegaP * dT * dT);
    psFilter->dA2 = 1.0 - 2.0 * dDp / dOmegaP / dT + 4.0 / (dOmegaP * dOmegaP * dT * dT);
}

double NotchTFUpdate(SNotchTF* psFilter, double dInput)
{
    double dOutput;
    
    // 递推计算公式
    dOutput = (psFilter->dB0 * dInput + 
               psFilter->dB1 * psFilter->dInPrev[0] + 
               psFilter->dB2 * psFilter->dInPrev[1] - 
               psFilter->dA1 * psFilter->dOutPrev[0] - 
               psFilter->dA2 * psFilter->dOutPrev[1]) / psFilter->dA0;
    
    // 更新历史数据
    psFilter->dInPrev[1] = psFilter->dInPrev[0];
    psFilter->dInPrev[0] = dInput;
    psFilter->dOutPrev[1] = psFilter->dOutPrev[0];
    psFilter->dOutPrev[0] = dOutput;
    
    return dOutput;
}

void NotchTFReset(SNotchTF* psFilter)
{
    // 清除历史数据
    psFilter->dOutPrev[0] = 0.0;
    psFilter->dOutPrev[1] = 0.0;
    psFilter->dInPrev[0] = 0.0;
    psFilter->dInPrev[1] = 0.0;
}