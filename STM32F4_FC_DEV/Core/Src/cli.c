/*
 * cli.c
 *
 *  Created on: Jul 21, 2022
 *      Author: 유지현
 */


#include <stdio.h>
#include <string.h>
#include "cli.h"
#include "main.h"



char cmd_buf[MAX_CMD_BUFFER_LENGTH];
char tmp_buf[MAX_CMD_BUFFER_LENGTH];


void CliInit(UART_HandleTypeDef* huart)
{
	QueueInit(&que);
//	UartInit(&huart);
	CommandInit();
	CliDisplayInfo();
    USART_Transmit(huart->Instance, "CMD>>",5);

}


void CliDisplayInfo()
{
	printf("===================================================================\r\n");
	printf("*                                                                 *\r\n");
	printf("*                                                                 *\r\n");
	printf("*                       STM32F405 CLI Program                     *\r\n");
	printf("*                       Version: 1.00                             *\r\n");
	printf("*                       Released: 2023-03-11                      *\r\n");
	printf("*                                                                 *\r\n");
	printf("*                                                                 *\r\n");
	printf("===================================================================\r\n");
}


void CliDo(UART_HandleTypeDef* huart)
{
	memset(cmd_buf, 0x00, MAX_CMD_BUFFER_LENGTH);

	RxCpltCallback(huart);

	if(UartReceiveBuffer(huart, (uint8_t *)cmd_buf, MAX_CMD_BUFFER_LENGTH)>0){
		CLiParseCmdLine(huart, cmd_buf);
	    USART_Transmit(huart->Instance, "CMD>>",5);
	}
}


void CLiParseCmdLine(UART_HandleTypeDef* huart, char* cmd_line)
{
	static const char *delim = " \f\n\r\t\v";
	char* tok;
	char* next_ptr;
	char *argv[5]= {0,};
	uint8_t argc = 0;

	//char tmp_line[MAX_CMD_BUFFER_LENGTH];
	memset(tmp_buf, 0x00, sizeof(tmp_buf));

	memcpy(tmp_buf, cmd_line, strlen(cmd_line));

	tok = strtok_r(tmp_buf, delim, &next_ptr);

	while(tok){
		argv[argc++] = tok;
		tok = strtok_r(NULL, delim, &next_ptr);
	}

	CliMatchCommand(argc, argv);
}


void CliMatchCommand(uint8_t argc, char* argv[])
{
	CommnadDo(argc, argv);
}






