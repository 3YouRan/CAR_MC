/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *5
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "PS2.h"
#include <string.h>
#include "math.h"
#include "retarget.h"
#include "PID.h"
#include "PID_Adjust.h"
#include "Motor.h"
#include "PS2.h"

//#include "../User/PS2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FORWARD 5
#define LEFT 8
#define BACK 7
#define RIGHT 6
#define A_ROUND_COUNT (74.8*360*4)
#define PI 3.1415926

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//����1���ջ�����
uint16_t RxLine = 0;//指令长度
uint8_t RxBuffer[1];//串口接收缓冲
uint8_t DataBuff[200];//指令内容
//λ�û�PID�ṹ��
PID pid_position;
//�����ٶȻ�PID�ṹ��
PID pid_speed;
PID pid_speed_B;
PID pid_speed_C;
PID pid_speed_D;
//�ǶȻ�PID�ṹ��
PID pid_angle;
//Ŀ��Ƕ�
float Target_Angle=0;
//����Ŀ���ٶ�
float Target_Speed=0;
float Target_Speed_A=0;
float Target_Speed_B=0;
float Target_Speed_C=0;
float Target_Speed_D=0;
//Ŀ����ٶ�
float Target_Angle_Speed=0;
//Ŀ��λ��
float Target_Position;
//������ṹ��
extern motor motorA;
extern motor motorB;
extern motor motorC;
extern motor motorD;
//������ͷ��ȡ������
float camera_data;
//�ֱ�����ָ������
uint8_t Key1;
//�˶�����
uint8_t direction;



void Kinematic_Analysis(float Vx,float Vy,float V_angle);
float transfer(int32_t x)//映射函数，将电机编码器的脉冲数转换为角度值（-180~180�????
{
    return (x/A_ROUND_COUNT)*360;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);//串口重定向初始化
  Motor_Init();//初始化电机结构体
  PID_Init(&pid_speed,&pid_speed_B,&pid_speed_C,&pid_speed_D,&pid_position,&pid_angle);//初始化pid结构�???
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);//打开串口1的接收中�???
  HAL_TIM_Base_Start_IT(&htim9);//�???始TIM9的定时中�???
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_ALL);
  PS2_SetInit();
  //HAL_UART_Receive_IT(&huart6, &rx_byte, 1);//打开串口6的接收中�???

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//      PS2_ReadData();          //��ȡ����
//      Key1 = PS2_DataKey();       //��ȡ�ֱ���������
//
//      switch (Key1) {
//          case 5:direction=FORWARD;break;//ǰ������??
//          case 6:direction=RIGHT;break;//ǰ������??
//          case 7:direction=BACK;break;//ǰ������??
//          case 8:direction=LEFT;break;//ǰ������??
//          case 10:Target_Angle+=-5;
//              break;//L2��ʹС��˳ʱ����ת
//          case 9:Target_Angle+=5;
//              break;//R2��ʹС����ʱ����ת
//          case 13:Target_Speed = 0;break;//�ٶȵ�λ���ڣ�rpm
//          case 14:Target_Speed = 30;break;//�ٶȵ�λ���ڣ�rpm
//          case 15:Target_Speed = 60;break;
//          case 16:Target_Speed=150;break;
//              //default:Target_Speed=0;break;
//      }
//      PS2_ClearData();      //����ֱ�������������
      Target_Speed_A=Target_Speed;
      HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if(huart==&huart1)
    {
        RxLine++;                      //每接收到�???个数据，进入回调数据长度�???1
        DataBuff[RxLine-1]=RxBuffer[0];  //把每次接收到的数据保存到缓存数组
        if(RxBuffer[0]=='!')           //接收结束标志�???
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);

            USART_PID_Adjust(1);//数据解析和参数赋值函�???

            memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
            RxLine=0;  //清空接收长度
        }
        RxBuffer[0]=0;

        HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);//每接收一个数据，就打�???�???次串口中断接�???
    }
//    if (huart == &huart6) {
//        // 处理接收到的数据
//
//        if (FifoSize > Uart.UartFifo.Cnt)
//        {
//            Uart.UartFifo.RxBuf[Uart.UartFifo.In] = rx_byte;
//            if(++Uart.UartFifo.In >= FifoSize)
//            {
//                Uart.UartFifo.In = 0;
//            }
//            ++Uart.UartFifo.Cnt;
//        }
//
//        // 重新启用接收中断，以便继续接收数�???
//        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
//    }
}
//��Ƶ��
uint8_t time1=0;

float angleNow1=0;
float angleNow2=0;
float angleNow3=0;
float angleNow4=0;
short encoder_now1=0;
short encoder_now2=0;
short encoder_now3=0;
short encoder_now4=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == (&htim9))//1ms
    {

        encoder_now1=(short )(__HAL_TIM_GET_COUNTER(ENCODER1));
        encoder_now2=(short )(__HAL_TIM_GET_COUNTER(ENCODER2));
        encoder_now3=(short )(__HAL_TIM_GET_COUNTER(ENCODER3));
        encoder_now4=(short )(__HAL_TIM_GET_COUNTER(ENCODER4));
        motorA.totalCount+=encoder_now1;//�ۼ�������
        motorB.totalCount+=encoder_now2;
        motorC.totalCount+=encoder_now3;
        motorD.totalCount+=encoder_now4;
        __HAL_TIM_SET_COUNTER(ENCODER1,0);
        __HAL_TIM_SET_COUNTER(ENCODER2,0);
        __HAL_TIM_SET_COUNTER(ENCODER3,0);
        __HAL_TIM_SET_COUNTER(ENCODER4,0);

        motorA.speed=(float )encoder_now1/(4*20*13)*1000*60; ////速度测量rpm
        motorB.speed=(float )encoder_now2/(4*20*13)*1000*60; //rpm
        motorC.speed=(float )encoder_now3/(4*20*13)*1000*60; //rpm
        motorD.speed=(float )encoder_now4/(4*20*13)*1000*60;

        angleNow1= transfer(motorA.totalCount);//ת���ۼ�������Ϊ�Ƕ�
        angleNow2= transfer(motorB.totalCount);
        angleNow3= transfer(motorC.totalCount);
        angleNow4= transfer(motorD.totalCount);

        time1++;
        if (time1==10){//10ms进行�???次PID计算
            time1=0;
            if (Target_Angle>180){
                Target_Angle=-180;
            }else if(Target_Angle<-180){
                Target_Angle=180;
            }
            printf("MOTOR:%.2f,%.2f,%.2f,%.2f,%.2f\n",pid_speed.kp,pid_speed.ki,pid_speed.kd,Target_Speed,motorA.speed);
            //Target_Speed= Position_PID_Realize(&pid_position, Target_Position, angleNow1);//位置�???
//            if(flag==-1) {//小车原地旋转模式
//                Target_Angle_Speed=Angle_PID_Realize(&pid_angle, Target_Angle, angle_Car);//角度�???
//
//            } else if(flag==1){
//                Angle_PID_Realize(&pid_angle, Target_Angle, angle_Car);//基于角度环的航向角修�???
//                switch (direction) {//通过全向轮运动模型设定不同运动状态下小车三电机的速度
//                    case FORWARD:Kinematic_Analysis(0, Target_Speed, pid_angle.output);break;
//                    case LEFT:Kinematic_Analysis(-Target_Speed, 0, pid_angle.output);break;
//                    case RIGHT:Kinematic_Analysis(Target_Speed, 0, pid_angle.output);break;
//                    case BACK:Kinematic_Analysis(0, -Target_Speed, pid_angle.output);break;
//                }
//            }
            Speed_PID_Realize(&pid_speed,Target_Speed_A,motorA.speed);//速度�???
            Speed_PID_Realize(&pid_speed_B,Target_Speed_B,motorB.speed);
            Speed_PID_Realize(&pid_speed_C,Target_Speed_C,motorC.speed);
            Speed_PID_Realize(&pid_speed_D,Target_Speed_D,motorD.speed);

            MotorA_Run(pid_speed.output);//PWM波输�???
            MotorB_Run(pid_speed_B.output);
            MotorC_Run(pid_speed_C.output);
            MotorD_Run(pid_speed_D.output);
        }



    }

}




#define RxPLUSRy 0.1
/**************************************************************************
�����ķ�����˶�ѧģ��
A B Ϊǰ����
C D Ϊ������
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float V_angle)
{
    Target_Speed_A=Vx-Vy-V_angle*RxPLUSRy;//��ǰ
    Target_Speed_B=Vx+Vy+V_angle*RxPLUSRy;//��ǰ
    Target_Speed_C=Vx+Vy-V_angle*RxPLUSRy;//���
    Target_Speed_D=Vx-Vy+V_angle*RxPLUSRy;//�Һ�
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

