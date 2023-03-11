/*
 * cli_uart.h
 *
 *  Created on: Jul 21, 2022
 *      Author: 유지현
 */

#ifndef INC_CLI_UART_H_
#define INC_CLI_UART_H_


#include "stm32f4xx_hal.h"
#include "define.h"
#include "cli_command.h"

//extern UART_HandleTypeDef huart4;

enum
{
  UART_RX_IDLE,
  UART_RX_ESC,
  UART_RX_BRACKET,
  UART_RX_DIR,
  UART_RX_TAB,
};


void UartInit(UART_HandleTypeDef* huart);
int UartReceiveBuffer(UART_HandleTypeDef* huart, uint8_t *buf, uint8_t size);
int UartTransmitBuffer(UART_HandleTypeDef* huart, uint8_t *buf, uint8_t size);
void UartWriteCmdBuffer(UART_HandleTypeDef* huart, int* uart_state);
void UartWriteSpecialKey(UART_HandleTypeDef* huart, int* uart_state);
void UartWriteDirKey(UART_HandleTypeDef* huart);
void UartWriteTabKey(UART_HandleTypeDef* huart);
void UartEraseCmdBuffer(UART_HandleTypeDef* huart);
void UartWriteHistoryOnCmdBuffer(UART_HandleTypeDef* huart);
void RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_CLI_UART_H_ */
