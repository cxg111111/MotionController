#ifndef CONTROLLED_DEVICE_H
#define CONTROLLED_DEVICE_H

#include <stdint.h>

// 刚体传递函数结构体
typedef struct {
    double in_prev[2];   // u[k-1], u[k-2]
    double out_prev[2];  // y[k-1], y[k-2]
    double a0, a1, a2;
    double b0, b1, b2;
    double mass;         // 质量参数
    double Ts;           // 采样时间
} RigidBodyTF;

// 函数声明
void RigidBodyTFInit(RigidBodyTF* tf, double mass, double dt);
double RigidBodyTFUpdate(RigidBodyTF* tf, double force);

#endif