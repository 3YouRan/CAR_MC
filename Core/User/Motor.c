//
// Created by 陈瑜 on 2024-02-07.
//
#include "Motor.h"

motor motorA;
motor motorB;
motor motorC;
motor motorD;
void Motor_Init(){
    HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);

    //MotorA
    motorA.lastCount = 0;                                   //结构体内容初始化
    motorA.totalCount = 0;
    motorA.overflowNum = 0;
    motorA.speed = 0;
    motorA.direct = 0;
    //MotorB
    motorB.lastCount = 0;
    motorB.totalCount = 0;
    motorB.overflowNum = 0;
    motorB.speed = 0;
    motorB.direct = 0;
    //MotorC
    motorC.lastCount = 0;
    motorC.totalCount = 0;
    motorC.overflowNum = 0;
    motorC.speed = 0;
    motorC.direct = 0;
}
void MotorA_Run(int Val){
    if(Val>=0){
        HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,SET);
        PWMA_SET(Val);
    }else{
        HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,RESET);

        PWMA_SET(-Val);
    }

}
void MotorB_Run(int16_t Val){
    if(Val>=0){
          HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,SET);
        PWMB_SET(Val);
    }else  if(Val<0){
        HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,RESET);
        PWMB_SET(-Val);
    }
}
void MotorC_Run(int16_t Val){
    if(Val>=0){
        HAL_GPIO_WritePin(CIN1_GPIO_Port,CIN1_Pin,SET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port,CIN2_Pin,RESET);
        PWMC_SET(Val);
    }else  if(Val<0){
        HAL_GPIO_WritePin(CIN1_GPIO_Port,CIN1_Pin,RESET);
        HAL_GPIO_WritePin(CIN2_GPIO_Port,CIN2_Pin,SET);
        PWMC_SET(-Val);
    }


}
void MotorD_Run(int16_t Val){
    if(Val>=0){
        HAL_GPIO_WritePin(DIN1_GPIO_Port,DIN1_Pin,SET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port,DIN2_Pin,RESET);
        PWMD_SET(Val);
    }else  if(Val<0){
        HAL_GPIO_WritePin(DIN1_GPIO_Port,DIN1_Pin,RESET);
        HAL_GPIO_WritePin(DIN2_GPIO_Port,DIN2_Pin,SET);
        PWMD_SET(-Val);
    }


}



