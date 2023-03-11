/*
 * define.h
 *
 *  Created on: Jul 21, 2022
 *      Author: 유지현
 */

#ifndef INC_DEFINE_H_
#define INC_DEFINE_H_


#define MAX_HISTORY_BUFFER_LENGTH	(10) /* MAX history buffer size in CLI */
#define MAX_CMD_BUFFER_LENGTH 		(50) /* MAX characters in one command line */
#define MAX_CMD_NUM 				(50) /* THE MAX NUMBER OF COMMANDS */


#define BL_ADDR_START				0x08000000
#define BL_ADDR_END					(BL_ADDR_START + (128 * 1024) - 1)


#define FW_ADDR_START				0x08020000
#define FW_ADDR_END					(FW_ADDR_START + (512 - 128) * 1024 - 1)

#endif /* INC_DEFINE_H_ */
