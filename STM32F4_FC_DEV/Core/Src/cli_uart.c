/*
 * cli_uart->c
 *
 *  Created on: Jul 21, 2022
 *      Author: 유지현
 */

#include <stdio.h>
#include <string.h>

#include "circular_queue.h"
#include "cli_uart.h"
#include "key_code.h"

/* current command line buffer */
uint8_t cmd_buff[MAX_CMD_BUFFER_LENGTH + 1];

int uart_state 				= UART_RX_IDLE;
uint8_t cmd_buff_length 	= 0;
extern uint8_t g_rx_buffer 		= 0;

Queue que;

void UartInit(UART_HandleTypeDef* huart)
{
	huart->Instance = USART6;
	huart->Init.BaudRate = 115200;
	huart->Init.WordLength = UART_WORDLENGTH_8B;
	huart->Init.StopBits = UART_STOPBITS_1;
	huart->Init.Parity = UART_PARITY_NONE;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart->Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(huart) != HAL_OK){
	  //Error_Handler();
	}

	HAL_UART_Receive_IT(huart, &g_rx_buffer, 1);
}


int UartReceiveBuffer(UART_HandleTypeDef* huart, uint8_t *buf, uint8_t size)
{
	int ret_value = 0;

	if(cmd_buff[cmd_buff_length-2] == '\r' && cmd_buff[cmd_buff_length-1] == '\n'){


		__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
		if(size < cmd_buff_length)
			cmd_buff_length = size;


		cmd_buff_length -= 2;
		memcpy(buf, cmd_buff, cmd_buff_length);
		ret_value = cmd_buff_length;

		if(cmd_buff_length > 0){
			Enqueue(&que, buf, cmd_buff_length);
			que.cursor = que.rear;
		}
		else if(cmd_buff_length==0) {
			USART_Transmit(USART6, "CMD>>",5);
		}

		memset(cmd_buff, 0x00, sizeof(cmd_buff));
		cmd_buff_length = 0;

		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
		HAL_UART_Receive_IT(huart, &g_rx_buffer, 1);

	}

	return ret_value;
}


int UartTransmitBuffer(UART_HandleTypeDef* huart, uint8_t *buf, uint8_t size){
	 if(HAL_UART_Transmit(huart, buf, size, 10) != HAL_OK)
	        return -1;

	    return 0;
}


void UartWriteCmdBuffer(UART_HandleTypeDef* huart, int* uart_state)
{
	char cdata[4], len = 1;
	memset(cdata, 0x00, 4);

	if(*uart_state == UART_RX_IDLE){
		switch(g_rx_buffer) {
			case CLI_KEY_ESC:
				*uart_state = UART_RX_ESC;
				break;

			case CLI_KEY_HORIZONTAL_TAB:
				*uart_state = UART_RX_TAB;
				break;

			case CLI_KEY_CARRIAGE_RETURN:
			case CLI_KEY_LINE_FEED:
				len = 2;
				sprintf(cdata, "\r\n");

				cmd_buff[cmd_buff_length++] = '\r';
				cmd_buff[cmd_buff_length++] = '\n';
				break;

			case '\b':
			case CLI_KEY_DELETE:
				if(cmd_buff_length == 0){
					break;
				}
				len = 3;
				sprintf(cdata, "\b \b");

				if(cmd_buff_length > 0)
					cmd_buff[cmd_buff_length--] = '\0';
				break;

			default:
				cdata[0] = g_rx_buffer;
				cmd_buff[cmd_buff_length++] = g_rx_buffer;
				break;
		}
	}

	UartWriteSpecialKey(huart, uart_state);

	HAL_UART_Transmit(huart, (uint8_t *)cdata, len, 10);
	HAL_UART_Receive_IT(huart, &g_rx_buffer, 1);
}


void UartWriteSpecialKey(UART_HandleTypeDef* huart, int* uart_state)
{

	switch(*uart_state){
		case UART_RX_ESC:
			*uart_state = UART_RX_BRACKET;
			HAL_UART_Receive_IT(huart, &g_rx_buffer, 1);
			break;

		case UART_RX_BRACKET:
			*uart_state = UART_RX_DIR;
			HAL_UART_Receive_IT(huart, &g_rx_buffer, 1);
			break;

		case UART_RX_DIR:
			*uart_state = UART_RX_IDLE;
			UartWriteDirKey(huart);
			break;

		case UART_RX_TAB:
			*uart_state = UART_RX_IDLE;
			UartWriteTabKey(huart);
			break;

		default:
			;

	}
}


void UartWriteDirKey(UART_HandleTypeDef* huart)
{
	if(g_rx_buffer == CLI_KEY_UP){
		if(que.cursor == que.front){
			return;
		}

		UartEraseCmdBuffer(huart);
		move_cursor(&que, UP);
		UartWriteHistoryOnCmdBuffer(huart);

	}
	else if(g_rx_buffer == CLI_KEY_DOWN){
		if(((que.cursor + 1)%MAX_HISTORY_BUFFER_LENGTH) == que.rear){
			return;
		}

		UartEraseCmdBuffer(huart);
		move_cursor(&que, DOWN);
		UartWriteHistoryOnCmdBuffer(huart);
	}
}


void UartWriteTabKey(UART_HandleTypeDef* huart)
{
	char tab_buff[MAX_CMD_BUFFER_LENGTH];
	memset(tab_buff, 0x00, sizeof(tab_buff));

	int total_commands = FindCommands((char *)cmd_buff, tab_buff);

	if(total_commands == 0){
		;
	}
	else if(total_commands== 1){
		UartEraseCmdBuffer(huart);

		cmd_buff_length = strlen(tab_buff);
		//HAL_UART_Transmit_IT(huart, (uint8_t *)tab_buff, cmd_buff_length);
		HAL_UART_Transmit(huart, (uint8_t *)tab_buff, cmd_buff_length, 100);
		memcpy(cmd_buff, (uint8_t *)tab_buff, cmd_buff_length);
	}
	else{
		UartEraseCmdBuffer(huart);

		memset(cmd_buff, 0x00, sizeof(cmd_buff));
		cmd_buff_length = 2;
		cmd_buff[0] = '\r';
		cmd_buff[1] = '\n';
	}
}


void UartEraseCmdBuffer(UART_HandleTypeDef* huart)
{
	char cdata[4];
	int len;

	memset(cdata, 0x00, 4);

	while(cmd_buff_length){
		len = 3;
		sprintf(cdata, "\b \b");

		if(cmd_buff_length)
			cmd_buff[--cmd_buff_length] = '\0';

		HAL_UART_Transmit(huart, (uint8_t *)cdata, len, 10);
		HAL_UART_Receive_IT(huart, &g_rx_buffer, 1);
	}
}


void UartWriteHistoryOnCmdBuffer(UART_HandleTypeDef* huart)
{
	memset(cmd_buff, 0x00, sizeof(cmd_buff));
	cmd_buff_length = strlen((const char *)que.history[que.cursor]);

	HAL_UART_Transmit(huart, (uint8_t *)que.history[que.cursor], cmd_buff_length, 10);
	memcpy(cmd_buff, (uint8_t *)que.history[que.cursor], cmd_buff_length);
}


void RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART6){
		UartWriteCmdBuffer(huart, &uart_state);
	}
}


