//
// Created by 陈瑜 on 2023-12-22.
//


#ifndef _PID_H_
#define _PID_H_

#include "stm32f4xx.h"
//#include "encoder.h"
#include <stdio.h>
//#include "control.h"

//PID三个参数的值
#define KP_speed 2.78
#define KI_speed 0
#define KD_speed 11.344
#define KP_position 0
#define KI_position 0
#define KD_position 0

typedef struct _PID//PID参数结构体
{
    float kp,ki,kd;
    float err,lastErr;
    float integral,maxIntegral; //积分值
    float output,maxOutput;
}PID;

void PID_Init(PID *pid_speed,PID *pid_speed_B,PID *pid_speed_C,PID *pid_speed_D,PID *pid_position,PID *pid_angle);//PID参数初始化
float Speed_PID_Realize(PID* pid,float target,float feedback);//一次PID计算
float Position_PID_Realize(PID* pid, float target, float feedback);
float Angle_PID_Realize(PID* pid, float target, float feedback);//一次PID计算
#endif
