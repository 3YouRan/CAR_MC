//
// Created by 陈瑜 on 2023-12-22.
//

#include "PID.h"
#include "pid.h"

/**********************************
 * 功能：PID结构体参数初始化
 * 输入：无
 * 返回：无
 * *******************************/
void PID_Init(PID *pid_speed,PID *pid_speed_B,PID *pid_speed_C,PID *pid_speed_D,PID *pid_position,PID *pid_angle)//PID参数初始化
{
    pid_speed->err = 0;
    pid_speed->integral = 0;
    pid_speed->maxIntegral = 5000;
    pid_speed->maxOutput=10000;
    pid_speed->lastErr = 0;
    pid_speed->output = 0;
    pid_speed->kp = KP_speed;
    pid_speed->ki = KI_speed;
    pid_speed->kd = KD_speed;

    pid_speed_B->err = 0;
    pid_speed_B->integral = 0;
    pid_speed_B->maxIntegral = 500;
    pid_speed_B->maxOutput=10000;
    pid_speed_B->lastErr = 0;
    pid_speed_B->output = 0;
    pid_speed_B->kp = KP_speed-0.5;
    pid_speed_B->ki = KI_speed;
    pid_speed_B->kd = KD_speed;

    pid_speed_C->err = 0;
    pid_speed_C->integral = 0;
    pid_speed_C->maxIntegral = 5000;
    pid_speed_C->maxOutput=10000;
    pid_speed_C->lastErr = 0;
    pid_speed_C->output = 0;
    pid_speed_C->kp = KP_speed;
    pid_speed_C->ki = KI_speed;
    pid_speed_C->kd = KD_speed;

    pid_speed_D->err = 0;
    pid_speed_D->integral = 0;
    pid_speed_D->maxIntegral = 5000;
    pid_speed_D->maxOutput=10000;
    pid_speed_D->lastErr = 0;
    pid_speed_D->output = 0;
    pid_speed_D->kp = KP_speed;
    pid_speed_D->ki = KI_speed;
    pid_speed_D->kd = KD_speed;

    pid_position->err = 0;
    pid_position->integral = 0;
    pid_position->maxIntegral = 800;
    pid_position->maxOutput =10000;
    pid_position->lastErr = 0;
    pid_position->output = 0;
    pid_position->kp = KP_position;
    pid_position->ki = KI_position;
    pid_position->kd = KD_position;

    pid_angle->err = 0;
    pid_angle->integral = 0;
    pid_angle->maxIntegral = 50;
    pid_angle->maxOutput =130;
    pid_angle->lastErr = 0;
    pid_angle->output = 0;
    pid_angle->kp = 1.09;
    pid_angle->ki = 0;
    pid_angle->kd = 0;
}

/****************************************
 * 作用：速度环PID计算
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：无
 * ****************************************/
float Speed_PID_Realize(PID* pid,float target,float feedback)//一次PID计算
{
    pid->err = target - feedback;
    if(pid->err < 0.01 && pid->err > -0.01) pid->err = 0;//pid死区
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    if(target == 0) pid->integral = 0; // 刹车时清空i


    pid->output += (pid->kp * pid->err) + (pid->ki * pid->integral)
                   + (pid->kd * (pid->err - pid->lastErr));//增量式PID

    //输出限幅
    if(target >= 0)//正转时
    {
        if(pid->output < 0) pid->output = 0;
        else if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    }
    else if(target < 0)//反转时
    {
        if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
        else if(pid->output > 0) pid->output = 0;
    }

    pid->lastErr = pid->err;
    if(target == 0) pid->output = 0; // 刹车时直接输出0
    return pid->output;
}
/****************************************
 * 作用：位置环PID计算
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：无
 * ****************************************/
float Position_PID_Realize(PID* pid, float target, float feedback)//一次PID计算
{

    if(pid->err < 0.1 && pid->err > -0.1) pid->err = 0;//pid死区
    pid->err = target - feedback;
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;

    return pid->output;
}

/****************************************
 * 作用：角度环PID计算
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：无
 * ****************************************/
float Angle_PID_Realize(PID* pid, float target, float feedback)//一次PID计算
{

    if(target-feedback>180){
        feedback+=360;
    }else if(target-feedback<-180){
        feedback-=360;
    }
    if(pid->err < 0.1 && pid->err > -0.1) pid->err = 0;//pid死区
    pid->err = target - feedback;
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    pid->output = (pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;

    return pid->output;
}

