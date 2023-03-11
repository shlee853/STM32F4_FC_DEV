/*
 * cli.h
 *
 *  Created on: Jul 21, 2022
 *      Author: 유지현
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_


#include "cli_command.h"
#include "cli_uart.h"


void CliInit(UART_HandleTypeDef* uart);
void CliDisplayInfo();
void CliDo(UART_HandleTypeDef* uart);
void CLiParseCmdLine(UART_HandleTypeDef* uart, char* cmd_line);
void CliMatchCommand(uint8_t argc, char* argv[]);



#endif /* INC_CLI_H_ */
