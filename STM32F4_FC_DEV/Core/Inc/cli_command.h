/*
 * command.h
 *
 *  Created on: 2022. 7. 22.
 *      Author: 유지현
 */

#ifndef INC_CLI_COMMAND_H_
#define INC_CLI_COMMAND_H_


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "main.h"
#include "define.h"
#include "circular_queue.h"


typedef struct CommandSet{
	char command_name[MAX_CMD_BUFFER_LENGTH];
	void (*CommandFuncPtr)(uint8_t argc, char * argv[]);
}CommandSet;


void CommandInit();
void CommnadDo(uint8_t argc, char* argv[]);
void CommandHistory(uint8_t argc, char* argv[]);
void CommandHelp(uint8_t argc, char* argv[]);
void CommandMd(uint8_t argc, char* argv[]);
void CommandLed(uint8_t argc, char* argv[]);
void CommandFwDownload(uint8_t argc, char* argv[]);
void CommandRunApplication(uint8_t argc, char* argv[]);
void CommandErase(uint8_t argc, char* argv[]);
int FindCommands(char* cmd_buff, char* tab_buff);


#endif /* INC_CLI_COMMAND_H_ */
