/*
 * command.c
 *
 *  Created on: 2022. 7. 22.
 *      Author: 유지현
 */

#include "main.h"
#include "cli_command.h"
#include "menu.h"
#include "cli_uart.h"

int usage = 0;

char commands[MAX_CMD_NUM][MAX_CMD_BUFFER_LENGTH] =
{
		"history",
		"help",
		"md",
		"led",
		"download",
		"run",
		"erase"
};
CommandSet commandSet[MAX_CMD_NUM];


void CommandInit()
{
	for(int i = 0; i< MAX_CMD_NUM; ++i){
		if(strlen(commands[i]) == 0){
			break;
		}
		memcpy(commandSet[i].command_name, commands[i], MAX_HISTORY_BUFFER_LENGTH);
		usage++;
	}

	commandSet[0].CommandFuncPtr = CommandHistory;
	commandSet[1].CommandFuncPtr = CommandHelp;
	commandSet[2].CommandFuncPtr = CommandMd;
	commandSet[3].CommandFuncPtr = CommandLed;
	commandSet[4].CommandFuncPtr = CommandFwDownload;
	commandSet[5].CommandFuncPtr = CommandRunApplication;
	commandSet[6].CommandFuncPtr = CommandErase;

}


void CommnadDo(uint8_t argc, char* argv[])
{
	for(int i = 0; i < usage; ++i){
		if(stricmp(argv[0], commandSet[i].command_name) == 0){
			commandSet[i].CommandFuncPtr(argc, argv);
			break;
		}
	}
}


void CommandHistory(uint8_t argc, char* argv[])
{
	if(argc > 2){
		printf("Usage : history [num]\r\n");
		return;
	}


	int num = atoi(argv[1]);
	print_history(&que, num);
}


void CommandHelp(uint8_t argc, char* argv[])
{
	printf("=====================================\r\n");

	for(int i = 0; i< usage; ++i){
		printf("* %s\r\n", commandSet[i].command_name);
	}

	printf("=====================================\r\n");

}


void CommandMd(uint8_t argc, char* argv[])
{
	int size = 0;
	unsigned int *addr, *cp_addr;
	unsigned char asc[4];

	if (argc != 3){
		printf("Usage : md addr [size] \r\n");
		return;
	}

	size = (int)strtoul((const char *)argv[2], (char**)NULL, (int)0);
	addr   = (unsigned int *)strtoul((const char * ) argv[1], (char **)NULL, (int) 0);
	cp_addr = (unsigned int *)addr;

	printf("\r\n");

	for (int idx = 0; idx < size; ++idx){
		if((idx%4) == 0){
			printf(" 0x%08X: ", (unsigned int)addr);
		}
		printf(" 0x%08X", *(addr));


		if ((idx%4) == 3)
		{
		  printf ("  |");
		  for (int idx1= 0; idx1< 4; idx1++)
		  {
			memcpy((char *)asc, (char *)cp_addr, 4);
			for (int i = 0; i < 4; ++i)
			{
			  if (asc[i] > 0x1f && asc[i] < 0x7f)
			  {
				printf("%c", asc[i]);
			  }
			  else
			  {
				printf(".");
			  }
			}
			cp_addr+=1;
		  }
		  printf("|\r\n");
		}
		addr++;
	}
}

void CommandFwDownload(uint8_t argc, char* argv[])
{
	// TO DO
	printf("BootLoader loacted in FLASH 0x%lx ~ 0x%lx\r\n", BOOTLOADER_ADDRESS, APPLICATION_ADDRESS - 1);
	printf("User Application will loacte in FLASH 0x%lx ~ 0x%x\r\n", APPLICATION_ADDRESS, USER_FLASH_END_ADDRESS);

	SerialDownload();
	UartInit(&huart6);

}


void CommandRunApplication(uint8_t argc, char* argv[])
{

	HAL_DeInit();
	RunApplication();
}

void CommandErase(uint8_t argc, char* argv[])
{
	FLASH_If_Erase(APPLICATION_ADDRESS);
}

void CommandLed(uint8_t argc, char* argv[])
{

	/*int led_high = GPIO_PIN_RESET;
	int led_low  = GPIO_PIN_SET;

	GPIO_TypeDef* port = LED1_GPIO_Port;
	uint16_t pin = LED1_Pin;

	if (argc != 3){
		printf("Usage : led [led_num] [On/Off]\n");
	}

	if(strcmp(argv[1], "1") == 0){
		if(stricmp(argv[2], "On") == 0){
			HAL_GPIO_WritePin(port, pin, led_low);
		}else if(stricmp(argv[2], "Off") == 0){
			HAL_GPIO_WritePin(port, pin, led_high);
		}else{
			printf("Usage : led [led_num] [On/Off]\n");
		}
	}else if(strcmp(argv[1], "2") == 0){
		port = LED2_GPIO_Port;
		pin = LED2_Pin;

		if(stricmp(argv[2], "On") == 0){
			HAL_GPIO_WritePin(port, pin, led_low);
		}else if(stricmp(argv[2], "Off") == 0){
			HAL_GPIO_WritePin(port, pin, led_high);
		}else{
			printf("Usage : led [led_num] [On/Off]\n");
		}
	}else{
		printf("Usage : led [led_num] [On/Off]\n");
	}*/
}



int FindCommands(char* cmd_buff, char* tab_buff)
{
	int count = 0, last_idx = 0;
	int cmd_buff_length = strlen(cmd_buff);

	char* check_list = (char *)malloc(sizeof(char) * usage);
	memset(check_list, 0x00, sizeof(char) * usage);

	if(cmd_buff_length == 0){
		return 0;
	}

	for(int i = 0; i < usage; ++i){
		if(strnicmp(cmd_buff, commands[i], cmd_buff_length) == 0){
			count++;
			last_idx = i;
			check_list[i] = 1;
		}
	}

	if(count == 0){
		;
	}
	else if (count == 1){
		memcpy(tab_buff, commands[last_idx], strlen(commands[last_idx]));
	}
	else{
		printf("\r\n");
		for(int i = 0; i < usage; ++i){
			if(check_list[i]){
				printf("* %s\r\n", commands[i]);
			}
		}
	}

	free(check_list);
	return count;
}
