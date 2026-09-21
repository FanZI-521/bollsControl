#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define CMD_LEN 255

extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCmd[CMD_LEN];
extern __IO uint8_t rxFrame[CMD_LEN];
extern __IO uint8_t rxCount;

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void Debug_UART_Print(const char *msg);

#ifdef __cplusplus
}
#endif

#endif
