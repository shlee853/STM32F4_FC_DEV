/*
 * SBUS_Receiver.h
 *
 *  Created on: Feb 22, 2023
 *      Author: swift
 */

#ifndef INC_SBUS_RECEIVER_H_
#define INC_SBUS_RECEIVER_H_


	#include "main.h"

	#define MSG_LENGTH_SBUS 		25
	#define MSG_SBUS_SOF 			0x0F
	#define MSG_SBUS_EOF 			0x00
	#define MSG_SBUS_CH01 			0x01
	#define MSG_SBUS_CH02 			0x02
	#define MSG_SBUS_CH03 			0x03
	#define MSG_SBUS_CH04 			0x04
	#define MSG_SBUS_CH05 			0x05
	#define MSG_SBUS_CH06 			0x06
	#define MSG_SBUS_CH07 			0x07
	#define MSG_SBUS_CH08 			0x08
	#define MSG_SBUS_CH09 			0x09
	#define MSG_SBUS_CH10 			0x0A
	#define MSG_SBUS_CH11 			0x0B
	#define MSG_SBUS_CH12 			0x0C
	#define MSG_SBUS_CH13 			0x0D
	#define MSG_SBUS_CH14 			0x0E
	#define MSG_SBUS_CH15 			0x0F
	#define MSG_SBUS_CH16 			0x10
	#define MSG_SBUS_CH17_BIT_MASK 	0x01
	#define MSG_SBUS_CH18_BIT_MASK 	0x02
	#define MSG_SBUS_FL_BIT_MASK 	0x04
	#define MSG_SBUS_FS_BIT_MASK 	0x08






	typedef struct{
		USART_TypeDef* UART;
		DMA_TypeDef* DMA;
		uint8_t rx_buf[MSG_LENGTH_SBUS];
	}SBUS_RAW_MESSAGE;

	typedef struct{
		uint8_t header;
		int16_t rx_channel[16];
		bool rx_channel17;
		bool rx_channel18;
		bool frame_lost;
		bool failsafe;
		uint8_t footer;
	}MSG_SBUS;



	void SBUS_Receiver_DMA_init(SBUS_RAW_MESSAGE* sbus_raw_message, USART_TypeDef* UART,DMA_TypeDef* DMA,uint32_t DMA_STREAM);
	void SBUS_Parsing(SBUS_RAW_MESSAGE* raw, MSG_SBUS* msg_sbus, int* rx_recv_cnt, int* rx_err_cnt);



#endif /* INC_SBUS_RECEIVER_H_ */
