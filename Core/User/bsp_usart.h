#ifndef __BSP_USART_H

#define __BSP_USART_H

#include "main.h"
#include "usart.h"


#define FifoSize 200   // 串口3的fifo大小 字节数

typedef struct // 串口驱动数据
{

    struct // 串口 Fifo缓冲区
    {
        uint8_t RxBuf[FifoSize];
        volatile uint32_t In;
        volatile uint32_t Out;
        volatile uint32_t Cnt;
    }UartFifo;

}struct_Ram_Uart;



extern uint8_t rx_byte;

int UART_Write(uint8_t n,  uint8_t *buf, int Len);





#endif

