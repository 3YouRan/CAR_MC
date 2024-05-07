//
// Created by 陈瑜 on 2024-02-07.
//

#ifndef CAR_ALL_ROUND_MOTOR_H
#define CAR_ALL_ROUND_MOTOR_H

#include "stm32f4xx.h"
#include "main.h"
#include "tim.h"
#define ENCODER1 &htim1
#define ENCODER2 &htim3
#define ENCODER3 &htim4
#define ENCODER4 &htim2
#define PWMA_SET(Val) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_4, Val); //Val(0~1000)
#define PWMB_SET(Val) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, Val);
#define PWMC_SET(Val) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_3, Val);
#define PWMD_SET(Val) __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_1, Val); //Val(0~1000)

typedef struct motor{
    int32_t lastCount;   //上一次计数值
    int32_t totalCount;  //总计数值
    int16_t overflowNum; //溢出次数
    float speed;         //电机转速
    uint8_t direct;      //旋转方向
}motor;

void Motor_Init();
void MotorA_Run(int Val);
void MotorB_Run(int16_t Val);
void MotorC_Run(int16_t Val);
void MotorD_Run(int16_t Val);
#endif //CAR_ALL_ROUND_MOTOR_H
