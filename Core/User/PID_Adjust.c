
#include "main.h"
#include "PID_Adjust.h"
#include "PID.h"
extern uint8_t RxBuffer[1];//串口接收缓冲
extern uint8_t DataBuff[200];//指令内容
//extern PID pid_r_position;
extern PID pid_position;
//extern PID pid_r_speed;
extern PID pid_angle;

extern float Target_Speed;
extern float Target_Angle;
//extern float R_Target_Speed;
extern float Target_Position;
//extern float R_Target_Position;

extern float camera_data;
/*
 * 解析出DataBuff中的数据
 * 返回解析得到的数据
 */
/*
 * 解析出DataBuff中的数据
 * 返回解析得到的数据
 */
float Get_Data(void)
{
    uint8_t data_Start_Num = 0; // 记录数据位开始的地方
    uint8_t data_End_Num = 0; // 记录数据位结束的地方
    uint8_t data_Num = 0; // 记录数据位数
    uint8_t minus_Flag = 0; // 判断是不是负数
    float data_return = 0; // 解析得到的数据
    for(uint8_t i=0;i<200;i++) // 查找等号和感叹号的位置
    {
        if(DataBuff[i] == '=') data_Start_Num = i + 1; // +1是直接定位到数据起始位
        if(DataBuff[i] == '!')
        {
            data_End_Num = i - 1;
            break;
        }
    }
    if(DataBuff[data_Start_Num] == '-') // 如果是负数
    {
        data_Start_Num += 1; // 后移一位到数据位
        minus_Flag = 1; // 负数flag
    }
    data_Num = data_End_Num - data_Start_Num + 1;
    if(data_Num == 4) // 数据共4位
    {
        data_return = (DataBuff[data_Start_Num]-48)  + (DataBuff[data_Start_Num+2]-48)*0.1f +
                      (DataBuff[data_Start_Num+3]-48)*0.01f;
    }
    else if(data_Num == 5) // 数据共5位
    {
        data_return = (DataBuff[data_Start_Num]-48)*10 + (DataBuff[data_Start_Num+1]-48) + (DataBuff[data_Start_Num+3]-48)*0.1f +
                      (DataBuff[data_Start_Num+4]-48)*0.01f;
    }
    else if(data_Num == 6) // 数据共6位
    {
        data_return = (DataBuff[data_Start_Num]-48)*100 + (DataBuff[data_Start_Num+1]-48)*10 + (DataBuff[data_Start_Num+2]-48) +
                      (DataBuff[data_Start_Num+4]-48)*0.1f + (DataBuff[data_Start_Num+5]-48)*0.01f;
    }
    if(minus_Flag == 1)  data_return = -data_return;
//    printf("data=%.2f\r\n",data_return);
    return data_return;
}

/*
 * 根据串口信息进行PID调参
 */
void USART_PID_Adjust(uint8_t Motor_n)
{
    float data_Get = Get_Data(); // 存放接收到的数据
//    printf("data=%.2f\r\n",data_Get);
    if(Motor_n == 1)//左边电机
    {
        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
            pid_position.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
            pid_position.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
            pid_position.kd = data_Get;
        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
            pid_angle.kp = data_Get;
        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
            pid_angle.ki = data_Get;
        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
            pid_angle.kd = data_Get;
        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
            Target_Angle = data_Get;
        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
            Target_Position = data_Get;
        else if((DataBuff[0]=='C' && DataBuff[1]=='A') && DataBuff[2]=='M') //摄像头数据
            camera_data = data_Get;
    }
//    else if(Motor_n == 0) // 右边电机
//    {
//        if(DataBuff[0]=='P' && DataBuff[1]=='1') // 位置环P
//            pid_r_position.kp = data_Get;
//        else if(DataBuff[0]=='I' && DataBuff[1]=='1') // 位置环I
//            pid_r_position.ki = data_Get;
//        else if(DataBuff[0]=='D' && DataBuff[1]=='1') // 位置环D
//            pid_r_position.kd = data_Get;
//        else if(DataBuff[0]=='P' && DataBuff[1]=='2') // 速度环P
//            pid_r_speed.kp = data_Get;
//        else if(DataBuff[0]=='I' && DataBuff[1]=='2') // 速度环I
//            pid_r_speed.ki = data_Get;
//        else if(DataBuff[0]=='D' && DataBuff[1]=='2') // 速度环D
//            pid_r_speed.kd = data_Get;
//        else if((DataBuff[0]=='S' && DataBuff[1]=='p') && DataBuff[2]=='e') //目标速度
//            R_Target_Speed = data_Get;
//        else if((DataBuff[0]=='P' && DataBuff[1]=='o') && DataBuff[2]=='s') //目标位置
//            R_Target_Position = data_Get;
//    }
}


