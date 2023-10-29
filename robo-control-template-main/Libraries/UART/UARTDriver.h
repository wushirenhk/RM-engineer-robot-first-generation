#ifndef __UART_DRIVER_H
#define __UART_DRIVER_H

#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

void USART1_DMA_Tx_Init(void);
void USART1_DMA_Send_String(const uint8_t * str);
void USART1_DMA_Send_Char(uint8_t * c);
void USART1_DMA_Debug_Printf(const char *format, ...);
void USART1_DMA_Debug_Plot(uint8_t name, int16_t value);


#endif