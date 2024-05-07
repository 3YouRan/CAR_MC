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
//串口1接收缓冲区
uint16_t RxLine = 0;//鎸囦护闀垮害
uint8_t RxBuffer[1];//涓插彛鎺ユ敹缂撳啿
uint8_t DataBuff[200];//鎸囦护鍐呭
//位置环PID结构体
PID pid_position;
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
//各电机结构体
extern motor motorA;
extern motor motorB;
extern motor motorC;
extern motor motorD;
//从摄像头获取的数据
float camera_data;
//手柄控制指令数据
uint8_t Key1;
//运动方向
uint8_t direction;



void Kinematic_Analysis(float Vx,float Vy,float V_angle);
float transfer(int32_t x)//鏄犲皠鍑芥暟锛屽皢鐢垫満缂栫爜鍣ㄧ殑鑴夊啿鏁拌浆鎹负瑙掑害鍊硷紙-180~180锟????
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
  RetargetInit(&huart1);//涓插彛閲嶅畾鍚戝垵濮嬪寲
  Motor_Init();//鍒濆鍖栫數鏈虹粨鏋勪綋
  PID_Init(&pid_speed,&pid_speed_B,&pid_speed_C,&pid_speed_D,&pid_position,&pid_angle);//鍒濆鍖杙id缁撴瀯锟???
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);//鎵撳紑涓插彛1鐨勬帴鏀朵腑锟???
  HAL_TIM_Base_Start_IT(&htim9);//锟???濮婽IM9鐨勫畾鏃朵腑锟???
  HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_ALL);
  PS2_SetInit();
  //HAL_UART_Receive_IT(&huart6, &rx_byte, 1);//鎵撳紑涓插彛6鐨勬帴鏀朵腑锟???

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
        RxLine++;                      //姣忔帴鏀跺埌锟???涓暟鎹紝杩涘叆鍥炶皟鏁版嵁闀垮害锟???1
        DataBuff[RxLine-1]=RxBuffer[0];  //鎶婃瘡娆℃帴鏀跺埌鐨勬暟鎹繚瀛樺埌缂撳瓨鏁扮粍
        if(RxBuffer[0]=='!')           //鎺ユ敹缁撴潫鏍囧織锟???
        {
            printf("RXLen=%d\r\n",RxLine);
            for(int i=0;i<RxLine;i++)
                printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);

            USART_PID_Adjust(1);//鏁版嵁瑙ｆ瀽鍜屽弬鏁拌祴鍊煎嚱锟???

            memset(DataBuff,0,sizeof(DataBuff));  //娓呯┖缂撳瓨鏁扮粍
            RxLine=0;  //娓呯┖鎺ユ敹闀垮害
        }
        RxBuffer[0]=0;

        HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);//姣忔帴鏀朵竴涓暟鎹紝灏辨墦锟???锟???娆′覆鍙ｄ腑鏂帴锟???
    }
//    if (huart == &huart6) {
//        // 澶勭悊鎺ユ敹鍒扮殑鏁版嵁
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
//        // 閲嶆柊鍚敤鎺ユ敹涓柇锛屼互渚跨户缁帴鏀舵暟锟???
//        HAL_UART_Receive_IT(&huart6, &rx_byte, 1);
//    }
}
//分频器
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
        motorA.totalCount+=encoder_now1;//累加脉冲数
        motorB.totalCount+=encoder_now2;
        motorC.totalCount+=encoder_now3;
        motorD.totalCount+=encoder_now4;
        __HAL_TIM_SET_COUNTER(ENCODER1,0);
        __HAL_TIM_SET_COUNTER(ENCODER2,0);
        __HAL_TIM_SET_COUNTER(ENCODER3,0);
        __HAL_TIM_SET_COUNTER(ENCODER4,0);

        motorA.speed=(float )encoder_now1/(4*20*13)*1000*60; ////閫熷害娴嬮噺rpm
        motorB.speed=(float )encoder_now2/(4*20*13)*1000*60; //rpm
        motorC.speed=(float )encoder_now3/(4*20*13)*1000*60; //rpm
        motorD.speed=(float )encoder_now4/(4*20*13)*1000*60;

        angleNow1= transfer(motorA.totalCount);//转换累计脉冲数为角度
        angleNow2= transfer(motorB.totalCount);
        angleNow3= transfer(motorC.totalCount);
        angleNow4= transfer(motorD.totalCount);

        time1++;
        if (time1==10){//10ms杩涜锟???娆ID璁＄畻
            time1=0;
            if (Target_Angle>180){
                Target_Angle=-180;
            }else if(Target_Angle<-180){
                Target_Angle=180;
            }
            printf("MOTOR:%.2f,%.2f,%.2f,%.2f,%.2f\n",pid_speed.kp,pid_speed.ki,pid_speed.kd,Target_Speed,motorA.speed);
            //Target_Speed= Position_PID_Realize(&pid_position, Target_Position, angleNow1);//浣嶇疆锟???
//            if(flag==-1) {//灏忚溅鍘熷湴鏃嬭浆妯″紡
//                Target_Angle_Speed=Angle_PID_Realize(&pid_angle, Target_Angle, angle_Car);//瑙掑害锟???
//
//            } else if(flag==1){
//                Angle_PID_Realize(&pid_angle, Target_Angle, angle_Car);//鍩轰簬瑙掑害鐜殑鑸悜瑙掍慨锟???
//                switch (direction) {//閫氳繃鍏ㄥ悜杞繍鍔ㄦā鍨嬭瀹氫笉鍚岃繍鍔ㄧ姸鎬佷笅灏忚溅涓夌數鏈虹殑閫熷害
//                    case FORWARD:Kinematic_Analysis(0, Target_Speed, pid_angle.output);break;
//                    case LEFT:Kinematic_Analysis(-Target_Speed, 0, pid_angle.output);break;
//                    case RIGHT:Kinematic_Analysis(Target_Speed, 0, pid_angle.output);break;
//                    case BACK:Kinematic_Analysis(0, -Target_Speed, pid_angle.output);break;
//                }
//            }
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




#define RxPLUSRy 0.1
/**************************************************************************
麦克纳姆轮逆运动学模型
A B 为前两轮
C D 为后两轮
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float V_angle)
{
    Target_Speed_A=Vx-Vy-V_angle*RxPLUSRy;//左前
    Target_Speed_B=Vx+Vy+V_angle*RxPLUSRy;//右前
    Target_Speed_C=Vx+Vy-V_angle*RxPLUSRy;//左后
    Target_Speed_D=Vx-Vy+V_angle*RxPLUSRy;//右后
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

