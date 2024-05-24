#include "bsp_usart.h"

#include "stdio.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

struct_Ram_Uart Uart;
//------------------------------------------------------------------------------
// 描述: Uart同步发送数据，等待发送完毕
// 输入: n=串口号, buf[Len]=要发送的内容
// 返回: 返回发送字节数
//------------------------------------------------------------------------------
int UART_Write(uint8_t n, uint8_t *buf, int Len)
{

    HAL_UART_Transmit(&huart6, buf, Len, 1000);

    return Len;
}







