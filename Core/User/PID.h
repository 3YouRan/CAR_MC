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
#define KP_speed 0
#define KI_speed 0
#define KD_speed 0
#define KP_position 1
#define KI_position 0
#define KD_position 1

typedef struct _PID//PID参数结构体
{
    float kp,ki,kd;
    float err,lastErr;
    float integral,maxIntegral; //积分值
    float output,maxOutput;
}PID;
extern PID pid_position_A;
extern PID pid_position_B;
extern PID pid_position_C;
extern PID pid_position_D;
void PID_Init(PID *pid_speed,PID *pid_speed_B,PID *pid_speed_C,PID *pid_speed_D,
              PID *pid_position,PID *pid_angle,PID *pid_Tracking,PID *pid_Tracking_Color,
              PID *pid_position_A,PID *pid_position_B,PID *pid_position_C,PID *pid_position_D);//PID参数初始化
float Speed_PID_Realize(PID* pid,float target,float feedback);//一次PID计算
float Position_PID_Realize(PID* pid, float target, float feedback);
float Angle_PID_Realize(PID* pid, float target, float feedback);//一次PID计算
int Get_Track_Data();
float Tracking_PID_Realize(PID* pid, float target, float feedback);//一次PID计算

#endif
