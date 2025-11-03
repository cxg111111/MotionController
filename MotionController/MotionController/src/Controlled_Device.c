#include "controlled_device.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// 初始化刚体传递函数
// Controlled_Device.c
void RigidBodyTFInit(RigidBodyTF *rb, double mass, double Ts) {
    
    // 初始化
    rb->in_prev[0] = 0.0;
    rb->in_prev[1] = 0.0;
    rb->out_prev[0] = 0.0;
    rb->out_prev[1] = 0.0;
    
    rb->mass = mass;
    rb->Ts = Ts;

    double T = Ts;
    double m = mass;

    // 离散化系数计算
    rb->b0 = T * T;
    rb->b1 = 2 * T * T;
    rb->b2 = T * T;

    rb->a0 = 4 * m;
    rb->a1 = -8 * m;
    rb->a2 = 4 * m;

}
// 更新函数
double RigidBodyTFUpdate(RigidBodyTF *rb, double input) {  

    double output = (rb->b0 * input +
                     rb->b1 * rb->in_prev[0] +
                     rb->b2 * rb->in_prev[1] -
                     rb->a1 * rb->out_prev[0] -
                     rb->a2 * rb->out_prev[1]) / rb->a0;

    // 更新历史值
    rb->in_prev[1] = rb->in_prev[0];
    rb->in_prev[0] = input;
    rb->out_prev[1] = rb->out_prev[0];
    rb->out_prev[0] = output;

    return output;

}

 
