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
#include "dma.h"
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
#include "im948_CMD.h"
#include "bsp_usart.h"
//#include "my_mpu6050.h"
//#include "inv_mpu.h"
//#include "../User/PS2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FORWARD 5
#define LEFT 8
#define BACK 7
#define RIGHT 6
#define A_ROUND_COUNT (13*20*4)
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
//串口1接收缓冲区
uint16_t RxLine = 0;//鎸囦护闀垮害
uint8_t RxBuffer[1];//涓插彛鎺ユ敹缂撳啿
uint8_t DataBuff[200];//鎸囦护鍐呭
//位置环PID结构体
PID pid_position;
PID pid_position_A;
PID pid_position_B;
PID pid_position_C;
PID pid_position_D;
//循迹PID结构体
PID pid_tracking;
//各轮速度环PID结构体
PID pid_speed;
PID pid_speed_B;
PID pid_speed_C;
PID pid_speed_D;
//角度环PID结构体
PID pid_angle;
//目标角度
float Target_Angle=0;
//各轮目标速度
float Target_Speed=0;
float Target_Speed_A=0;
float Target_Speed_B=0;
float Target_Speed_C=0;
float Target_Speed_D=0;
//目标角速度
float Target_Angle_Speed=0;
//目标位置
float Target_Position;
float Target_Position_A;
float Target_Position_B;
float Target_Position_C;
float Target_Position_D;
//各电机结构体
extern motor motorA;
extern motor motorB;
extern motor motorC;
extern motor motorD;
//从摄像头获取的数据
float camera_data[50];
uint8_t camera_data_len=0;
float camera_data_last=0;
uint8_t camera_flag=0;
float camera_data_get=0;
uint8_t rpt_time=0;
PID pid_Tracking_Color;
//手柄控制指令数据
uint8_t Key1;
//运动方向
uint8_t direction=LEFT;
//位置反馈
float angleNow1=0;
float angleNow2=0;
float angleNow3=0;
float angleNow4=0;
//循迹数据
uint16_t SENSOR[5];
//任务flag
uint8_t task_flag=0;
uint8_t a_times=10;
uint8_t Position_flag=0;
uint8_t Angle_flag=0;
//uint8_t b_times=5;
//陀螺仪数据
extern U8 Data[9];
extern U16 MASK[16];
extern U16 Handkey;
extern struct_Ram_Uart Uart;
float angle_Car;
float angle_Car_last;
float angle_Car_total;
uint8_t rx_byte;
float X_Car;
float Y_Car;

void Kinematic_Analysis(float Vx,float Vy,float V_angle);
void Kinematic_Analysis_Pos(float Target_Position_x,float Target_Position_y,float V_angle);
float transfer(int32_t x)//鏄犲皠鍑芥暟锛屽皢鐢垫満缂栫爜鍣ㄧ殑鑴夊啿鏁拌浆鎹负瑙掑害鍊硷紙-180~180锟????
{
    return ((float)x/A_ROUND_COUNT)*360;
}
void beep_on(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port,BUZZER_Pin,RESET);
}

void BUZZER_work (int time){
    while(time--)
    {
        beep_on();
        HAL_Delay(200);
    }
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  RetargetInit(&huart1);//涓插彛閲嶅畾鍚戝垵濮嬪寲
  Motor_Init();//鍒濆鍖栫數鏈虹粨鏋勪綋
  PID_Init(&pid_speed,&pid_speed_B,&pid_speed_C,&pid_speed_D,&pid_position,
           &pid_angle,&pid_tracking,&pid_Tracking_Color,&pid_position_A,&pid_position_B,&pid_position_C,&pid_position_D);//鍒濆鍖杙id缁撴瀯锟???
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);//鎵撳紑涓插彛1鐨勬帴鏀朵腑锟???
  HAL_TIM_Base_Start_IT(&htim10);//锟???濮婽IM9鐨勫畾鏃朵腑锟???
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)SENSOR, 5);
  HAL_UART_Receive_IT(&huart6, &rx_byte, 1);//打开串口6的接收中??

  //PS2_SetInit();
  //mpu_dmp_init();
//    uint8_t err = mpu_dmp_init();
//    while(err)
//    {
//        printf("mpu_init_err:%d\r\n",err);
//        err=mpu_dmp_init();
//    }
//    printf("mpu9250 Ok\r\n");
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
  HAL_Delay(1000);
    Cmd_03();//唤醒传感??
    Cmd_12(5, 255, 0,  0, 3, 200, 2, 4, 9, 0x40);//设置IM600设备参数
    Cmd_05();// 归零IM600Z轴姿态角数据，以小车复位时的姿???角为角??0??
    Cmd_19();// ??启数据主动上??
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      if (task_flag==1) {
          angle_Car_last=angle_Car;
      }
      U8 rxByte;
      while (Uart.UartFifo.Cnt > 0)
      {// 从fifo获取串口发来的数??

          rxByte = Uart.UartFifo.RxBuf[Uart.UartFifo.Out];
          if (++Uart.UartFifo.Out >= FifoSize)
          {
              Uart.UartFifo.Out = 0;
          }
          __disable_irq();
          --Uart.UartFifo.Cnt;
          __enable_irq();

          Cmd_GetPkt(rxByte); //每收??1字节数据都填入该函数，当抓取到有效的数据包就会回调进?? Cmd_RxUnpack(U8 *buf, U8 DLen) 函数处理
      }
//      PS2_ReadData();          //获取数据
//      Key1 = PS2_DataKey();       //获取手柄按键数据
//
//      switch (Key1) {
//          case 5:direction=FORWARD;break;//前进方向??
//          case 6:direction=RIGHT;break;//前进方向??
//          case 7:direction=BACK;break;//前进方向??
//          case 8:direction=LEFT;break;//前进方向??
//          case 10:Target_Angle+=-5;
//              break;//L2键使小车顺时针旋转
//          case 9:Target_Angle+=5;
//              break;//R2键使小车逆时针旋转
//          case 13:Target_Speed = 0;break;//速度档位调节，rpm
//          case 14:Target_Speed = 30;break;//速度档位调节，rpm
//          case 15:Target_Speed = 60;break;
//          case 16:Target_Speed=150;break;
//              //default:Target_Speed=0;break;
//      }
//      PS2_ClearData();      //清除手柄按键数据数据
      //Target_Speed_C=Target_Speed;
//        if(camera_flag!=1){
//            Target_Angle_Speed = 0;
//        }
      //HAL_Delay(50);
//      if (rpt_time==5) {
//          rpt_time=0;
//          camera_data = 0;
//          camera_flag=0;
//      }

      //状态1：循迹
      if(task_flag==0){

          Target_Speed = 88.1;
//          beep_on();
          if (((SENSOR[0]+SENSOR[1]+SENSOR[2]+SENSOR[3]+SENSOR[4])<3000)){
              HAL_ADC_Stop_DMA(&hadc1);
              Target_Speed = 0;
              Target_Angle_Speed=0;

              Kinematic_Analysis(Target_Speed,0, Target_Angle_Speed);
              beep_on();
              HAL_Delay(2000);
              Position_flag=1;
              direction=FORWARD;
              angleNow1=0;
              angleNow2=0;
              angleNow3=0;
              angleNow4=0;
              motorA.totalCount=0;
              motorB.totalCount=0;
              motorC.totalCount=0;
              motorD.totalCount=0;
              Target_Position=720;
              HAL_Delay(1000);
              Position_flag=0;
              task_flag=1;
          }
      }

      //状态2：物体检测
      if (task_flag==1){
          if (a_times>0) {
              printf("a");
              a_times--;
          }
          while(!camera_flag) {
              direction=LEFT;
              angleNow1=0;
              angleNow2=0;
              angleNow3=0;
              angleNow4=0;
              motorA.totalCount=0;
              motorB.totalCount=0;
              motorC.totalCount=0;
              motorD.totalCount=0;
              Target_Position=+30;
              Position_flag=1;

          }
          Position_flag=0;
          Target_Speed=0;
          Target_Angle_Speed=0;
          Kinematic_Analysis(Target_Speed,0, Target_Angle_Speed);
          if (((SENSOR[0]+SENSOR[1]+SENSOR[2]+SENSOR[3]+SENSOR[4])<000)){

              Target_Speed = 0;
              Kinematic_Analysis(Target_Speed,0, Target_Angle_Speed);

              HAL_Delay(2000);
              task_flag=2;
          }
      }
      //状态3：数字识辨
      if(task_flag==2) {
          printf("b");
          Kinematic_Analysis(0,0, 0);
//          if (b_times>0) {
//              printf("b");
//              b_times--;
//          }

          if (rpt_time >= 5) {
              int LED_times = (int) camera_data_get;
              for (int i = 0; i < LED_times; i++) {
                  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                  HAL_Delay(250);
                  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
                  HAL_Delay(250);
                  beep_on();
              }
              rpt_time = 0;
              HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
              HAL_Delay(1000);
          }
      }



      //Kinematic_Analysis_Pos(0,Target_Position,Target_Angle_Speed);


      printf("yaw: %.2f,%.2f,%.2f,%.2f,%.2f\n",motorA.speed,motorB.speed,angle_Car,X_Car,Y_Car);

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
        RxLine++;                      //姣忔帴鏀跺埌锟???涓暟鎹紝杩涘叆鍥炶皟鏁版嵁闀垮害锟???1
        DataBuff[RxLine-1]=RxBuffer[0];  //鎶婃瘡娆℃帴鏀跺埌鐨勬暟鎹繚瀛樺埌缂撳瓨鏁扮粍
        if(RxBuffer[0]=='!')           //鎺ユ敹缁撴潫鏍囧織锟???
        {
            //printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                //printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);

            USART_PID_Adjust(1);//鏁版嵁瑙ｆ瀽鍜屽弬鏁拌祴鍊煎嚱锟???

            memset(DataBuff,0,sizeof(DataBuff));  //娓呯┖缂撳瓨鏁扮粍
            RxLine=0;  //娓呯┖鎺ユ敹闀垮害
        }
        RxBuffer[0]=0;

        HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);//姣忔帴鏀朵竴涓暟鎹紝灏辨墦锟???锟???娆′覆鍙ｄ腑鏂帴锟???
    }

    if (huart == &huart6) {
        // 处理接收到的数据

        if (FifoSize > Uart.UartFifo.Cnt)
        {
            Uart.UartFifo.RxBuf[Uart.UartFifo.In] = rx_byte;
            if(++Uart.UartFifo.In >= FifoSize)
            {
                Uart.UartFifo.In = 0;
            }
            ++Uart.UartFifo.Cnt;
        }

        // 重新启用接收中断，以便继续接收数??
        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
    }
}
//分频器
uint8_t time1=0;
uint8_t time2=0;


short encoder_now1=0;
short encoder_now2=0;
short encoder_now3=0;
short encoder_now4=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == (&htim10))//1ms
    {
        time1++;
        time2++;
        if (time2==2){
            time2=0;
            encoder_now1=(short )(__HAL_TIM_GET_COUNTER(ENCODER1));
            encoder_now2=(short )(__HAL_TIM_GET_COUNTER(ENCODER2));
            encoder_now3=(short )(__HAL_TIM_GET_COUNTER(ENCODER3));
            encoder_now4=(short )(__HAL_TIM_GET_COUNTER(ENCODER4));
            motorA.totalCount+=encoder_now1;//累加脉冲数
            motorB.totalCount+=encoder_now2;
            motorC.totalCount+=encoder_now3;
            motorD.totalCount+=encoder_now4;
            __HAL_TIM_SET_COUNTER(ENCODER1,0);
            __HAL_TIM_SET_COUNTER(ENCODER2,0);
            __HAL_TIM_SET_COUNTER(ENCODER3,0);
            __HAL_TIM_SET_COUNTER(ENCODER4,0);

            motorA.speed=(float )encoder_now1/(4*20*13)*500*60; ////閫熷害娴嬮噺rpm
            motorB.speed=(float )encoder_now2/(4*20*13)*500*60; //rpm
            motorC.speed=(float )encoder_now3/(4*20*13)*500*60;
            motorD.speed=(float )encoder_now4/(4*20*13)*500*60;

            angleNow1= transfer(motorA.totalCount);//转换累计脉冲数为角度
            angleNow2= transfer(motorB.totalCount);
            angleNow3= transfer(motorC.totalCount);
            angleNow4= transfer(motorD.totalCount);

        }
        if (time1==5){//10ms杩涜锟???娆ID璁＄畻
            Kinematic_Analysis(Target_Speed,0, Target_Angle_Speed);

            time1=0;

            if (Target_Angle>180){
                Target_Angle=-180;
            }else if(Target_Angle<-180){
                Target_Angle=180;
            }


            if (task_flag==0) {
                Target_Angle_Speed = -Tracking_PID_Realize(&pid_tracking, 0, Get_Track_Data());
                //Target_Angle_Speed= Angle_PID_Realize(&pid_angle,Target_Angle, angle_Car);

            }
            if(Position_flag&&direction==LEFT){
                Kinematic_Analysis_Pos(0,Target_Position,0);
            }
            if(Position_flag&&direction==FORWARD){
                Kinematic_Analysis_Pos(Target_Position,0,0);
            }
            if (Angle_flag){
                Target_Angle_Speed = Angle_PID_Realize(&pid_angle,Target_Angle, angle_Car);
            }
            Speed_PID_Realize(&pid_speed,Target_Speed_A,motorA.speed);//閫熷害锟???
            Speed_PID_Realize(&pid_speed_B,Target_Speed_B,motorB.speed);
            Speed_PID_Realize(&pid_speed_C,Target_Speed_C,motorC.speed);
            Speed_PID_Realize(&pid_speed_D,Target_Speed_D,motorD.speed);

            MotorA_Run(pid_speed.output);//PWM娉㈣緭锟???
            MotorB_Run(pid_speed_B.output);
            MotorC_Run(pid_speed_C.output);
            MotorD_Run(pid_speed_D.output);

        }

    }

}




#define RxPLUSRy 0.12/2+0.16/2
/**************************************************************************
麦克纳姆轮逆运动学模型
A B 为前两轮
C D 为后两轮
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float V_angle)
{
    Target_Speed_A=-(Vx-Vy-V_angle*RxPLUSRy);//左前
    Target_Speed_B=-(Vx+Vy+V_angle*RxPLUSRy);//右前
    Target_Speed_C=-(Vx+Vy-V_angle*RxPLUSRy);//左后
    Target_Speed_D=-(Vx-Vy+V_angle*RxPLUSRy);//右后
}
/**************************************************************************
麦克纳姆轮逆运动学模型_位置环分配
A B 为前两轮
C D 为后两轮
**************************************************************************/
void Kinematic_Analysis_Pos(float Target_Position_x,float Target_Position_y,float V_angle)
{

        Target_Position_A =(Target_Position_x-Target_Position_y);//左前
        Target_Position_B =(Target_Position_x+Target_Position_y);//右前
        Target_Position_C =(Target_Position_x+Target_Position_y);//左后
        Target_Position_D =(Target_Position_x-Target_Position_y);//右后
        Target_Speed_A=-(Position_PID_Realize(&pid_position_A, Target_Position_A, -angleNow1));//左前
        Target_Speed_B=-(Position_PID_Realize(&pid_position_B, Target_Position_B, -angleNow2));//右前
        Target_Speed_C=-(Position_PID_Realize(&pid_position_C, Target_Position_C, -angleNow3));//左后
        Target_Speed_D=-(Position_PID_Realize(&pid_position_D, Target_Position_D, -angleNow4));//右后

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

