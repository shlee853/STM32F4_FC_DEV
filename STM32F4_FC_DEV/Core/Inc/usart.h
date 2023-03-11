/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */
#define CIRCULAR_BUFFER_SIZE 1024

enum{
	CIRCULAR_READY,
	CIRCULAR_BUSY
}state;

enum{
	CIRCULAR_REMAIN,
	CIRCULAR_FULL
}is_full;

enum{
	CIRCULAR_DATA_INCREASE,
	CIRCULAR_DATA_HOLD
}increase;

typedef struct{
	USART_TypeDef* UART;
	DMA_TypeDef* DMA;
	uint32_t DMA_CHANNEL_STREAM;
	uint8_t buffer[CIRCULAR_BUFFER_SIZE];
	uint8_t temp_buffer[CIRCULAR_BUFFER_SIZE];
	uint16_t start_position;
	uint16_t end_position;
	uint16_t temp_end_position;
	uint8_t is_full;
	uint8_t state;
}CIRCULAR_BUFFER;




/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
int _write(int file, char* p, int len);
void USART_Transmit(USART_TypeDef *USARTx, uint8_t * data, uint16_t length);
void PRINTF_INIT(USART_TypeDef* UART, DMA_TypeDef* DMA, uint32_t DMA_CHANNEL_STREAM);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

