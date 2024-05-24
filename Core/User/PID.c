//
// Created by 陈瑜 on 2023-12-22.
//

#include "PID.h"
#include "pid.h"
extern uint16_t SENSOR[5];
/**********************************
 * 功能：PID结构体参数初始化
 * 输入：无
 * 返回：无
 * *******************************/
void PID_Init(PID *pid_speed,PID *pid_speed_B,PID *pid_speed_C,PID *pid_speed_D,
              PID *pid_position,PID *pid_angle,PID *pid_Tracking,PID *pid_Tracking_Color,
              PID *pid_position_A,PID *pid_position_B,PID *pid_position_C,PID *pid_position_D)//PID参数初始化
{
    pid_speed->err = 0;
    pid_speed->integral = 0;
    pid_speed->maxIntegral = 5000;
    pid_speed->maxOutput=1000;
    pid_speed->lastErr = 0;
    pid_speed->output = 0;
    pid_speed->kp = 0.15;
    pid_speed->ki = KI_speed;
    pid_speed->kd = 1;

    pid_speed_B->err = 0;
    pid_speed_B->integral = 0;
    pid_speed_B->maxIntegral = 500;
    pid_speed_B->maxOutput=10000;
    pid_speed_B->lastErr = 0;
    pid_speed_B->output = 0;
    pid_speed_B->kp = 0.27;
    pid_speed_B->ki = KI_speed;
    pid_speed_B->kd = 1;

    pid_speed_C->err = 0;
    pid_speed_C->integral = 0;
    pid_speed_C->maxIntegral = 5000;
    pid_speed_C->maxOutput=10000;
    pid_speed_C->lastErr = 0;
    pid_speed_C->output = 0;
    pid_speed_C->kp = 0.15;
    pid_speed_C->ki = KI_speed;
    pid_speed_C->kd = 1;

    pid_speed_D->err = 0;
    pid_speed_D->integral = 0;
    pid_speed_D->maxIntegral = 5000;
    pid_speed_D->maxOutput=10000;
    pid_speed_D->lastErr = 0;
    pid_speed_D->output = 0;
    pid_speed_D->kp = 0.27;
    pid_speed_D->ki = KI_speed;
    pid_speed_D->kd = 1;

    pid_position->err = 0;
    pid_position->integral = 0;
    pid_position->maxIntegral = 800;
    pid_position->maxOutput =200;
    pid_position->lastErr = 0;
    pid_position->output = 0;
    pid_position->kp = 1;
    pid_position->ki = KI_position;
    pid_position->kd = 1;

    pid_angle->err = 0;
    pid_angle->integral = 0;
    pid_angle->maxIntegral = 50;
    pid_angle->maxOutput =5000;
    pid_angle->lastErr = 0;
    pid_angle->output = 0;
    pid_angle->kp = 30;
    pid_angle->ki = 0;
    pid_angle->kd = 0;

    pid_Tracking->err = 0;
    pid_Tracking->integral = 0;
    pid_Tracking->maxIntegral =0;
    pid_Tracking->maxOutput =1000;
    pid_Tracking->lastErr = 0;
    pid_Tracking->output = 0;
    pid_Tracking->kp = 70;
    pid_Tracking->ki = 0;
    pid_Tracking->kd = 150;

    pid_Tracking_Color->err = 0;
    pid_Tracking_Color->integral = 0;
    pid_Tracking_Color->maxIntegral =0;
    pid_Tracking_Color->maxOutput =1000;
    pid_Tracking_Color->lastErr = 0;
    pid_Tracking_Color->output = 0;
    pid_Tracking_Color->kp = 5;
    pid_Tracking_Color->ki = 0;
    pid_Tracking_Color->kd = 0;

    pid_position_A->err = 0;
    pid_position_A->integral = 0;
    pid_position_A->maxIntegral = 800;
    pid_position_A->maxOutput =100;
    pid_position_A->lastErr = 0;
    pid_position_A->output = 0;
    pid_position_A->kp = KP_position;
    pid_position_A->ki = KI_position;
    pid_position_A->kd = KD_position;

    pid_position_B->err = 0;
    pid_position_B->integral = 0;
    pid_position_B->maxIntegral = 800;
    pid_position_B->maxOutput =100;
    pid_position_B->lastErr = 0;
    pid_position_B->output = 0;
    pid_position_B->kp = KP_position;
    pid_position_B->ki = KI_position;
    pid_position_B->kd = KD_position;

    pid_position_C->err = 0;
    pid_position_C->integral = 0;
    pid_position_C->maxIntegral = 800;
    pid_position_C->maxOutput =100;
    pid_position_C->lastErr = 0;
    pid_position_C->output = 0;
    pid_position_C->kp = KP_position;
    pid_position_C->ki = KI_position;
    pid_position_C->kd = KD_position;

    pid_position_D->err = 0;
    pid_position_D->integral = 0;
    pid_position_D->maxIntegral = 800;
    pid_position_D->maxOutput =100;
    pid_position_D->lastErr = 0;
    pid_position_D->output = 0;
    pid_position_D->kp = KP_position;
    pid_position_D->ki = KI_position;
    pid_position_D->kd = KD_position;
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
#define L_SENSOR_MAX 3250
#define R_SENSOR_MAX 2950
#define LL_SENSOR_MAX 2200
#define RR_SENSOR_MAX 2250
#define L_LIMIT 1400
#define R_LIMIT 2900
int Get_Track_Data(){
    int Data_out;
    uint16_t LL_AD,L_AD,R_AD,M_AD,RR_AD;
    LL_AD=SENSOR[0];
    L_AD=SENSOR[1];
    M_AD=SENSOR[2];
    R_AD=SENSOR[3];
    RR_AD=SENSOR[4];

    Data_out=((2*LL_AD+L_AD)-(R_AD+2*RR_AD)+50-300);
    if(M_AD<800){
        Data_out=0;
    }
    if (Data_out>0&&M_AD<L_LIMIT){
        Data_out=(2*(2*LL_SENSOR_MAX+L_SENSOR_MAX)-(L_AD+2*LL_AD))*2;
    } else if (Data_out<0&&M_AD<R_LIMIT){
        Data_out=(R_AD+2*RR_AD-2*(R_SENSOR_MAX+2*RR_SENSOR_MAX))*2;
    }
    return Data_out;
}
/****************************************
 * 作用：循迹PID计算
 * 参数：PID参数结构体地址；目标值；反馈值
 * 返回值：无
 * ****************************************/
float Tracking_PID_Realize(PID* pid, float target, float feedback)//一次PID计算
{


    if(pid->err < 1000 && pid->err > -1000) pid->err = 0;//pid死区
    pid->err = target - feedback;
    pid->integral += pid->err;

    if(pid->ki * pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral / pid->ki;//积分限幅
    else if(pid->ki * pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral / pid->ki;

    pid->output = +(pid->kp * pid->err) + (pid->ki * pid->integral) + (pid->kd * (pid->err - pid->lastErr));//全量式PID

    //输出限幅
    if(pid->output > pid->maxOutput) pid->output = pid->maxOutput;
    if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;

    pid->lastErr = pid->err;

    return pid->output;
}
