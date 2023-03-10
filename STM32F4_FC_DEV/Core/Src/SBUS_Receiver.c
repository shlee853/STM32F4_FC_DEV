/*
 * SBUS_Receiver.c
 *
 *  Created on: Feb 22, 2023
 *      Author: swift
 */


#include "SBUS_Receiver.h"


	int rx_recv_cnt=0;
	int rx_err_cnt=0;


void SBUS_Receiver_DMA_init(SBUS_RAW_MESSAGE* raw, USART_TypeDef* UART,DMA_TypeDef* DMA,uint32_t DMA_STREAM)
{


	//DMA, INTERRUPT SETTINGS
	LL_DMA_SetMemoryAddress(DMA,DMA_STREAM,(uint32_t)(raw->rx_buf));
	LL_DMA_SetPeriphAddress(DMA,DMA_STREAM,LL_USART_DMA_GetRegAddr(UART));
	LL_DMA_SetDataLength(DMA,DMA_STREAM,MSG_LENGTH_SBUS);

	//  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TC(DMA, DMA_STREAM);
	LL_USART_EnableDMAReq_RX(UART);

}


void SBUS_Parsing(SBUS_RAW_MESSAGE* raw, MSG_SBUS* msg_sbus, int* rx_recv_cnt, int* rx_err_cnt)
{
	uint8_t* ptr = raw->rx_buf;
	int32_t temp;
	unsigned short msg_length = 0, checksum = 0;
	unsigned char classID = 0, messageID = 0;
	if(ptr[0]==MSG_SBUS_SOF ){
		msg_sbus->header = ptr[0];
	       msg_sbus->rx_channel[0]   = (int16_t)(ptr[1] | ((ptr[2] << 8) & 0x07FF));
	        msg_sbus->rx_channel[1]  = (int16_t)((ptr[2] >> 3) | ((ptr[3] << 5) & 0x07FF));
	        msg_sbus->rx_channel[2]  = (int16_t)((ptr[3] >> 6) | (ptr[4] << 2) | ((ptr[5] << 10) & 0x07FF));
	        msg_sbus->rx_channel[3]  = (int16_t)((ptr[5] >> 1) | ((ptr[6] << 7) & 0x07FF));
	        msg_sbus->rx_channel[4]  = (int16_t)((ptr[6] >> 4) | ((ptr[7] << 4) & 0x07FF));
	        msg_sbus->rx_channel[5]  = (int16_t)((ptr[7] >> 7) | (ptr[8] << 1) | ((ptr[9] << 9) & 0x07FF));
	        msg_sbus->rx_channel[6]  = (int16_t)((ptr[9] >> 2) | ((ptr[10] << 6) & 0x07FF));
	        msg_sbus->rx_channel[7]  = (int16_t)((ptr[10] >> 5) | ((ptr[11] << 3) & 0x07FF));
	        msg_sbus->rx_channel[8]  = (int16_t)(ptr[12] | ((ptr[13] << 8) & 0x07FF));
	        msg_sbus->rx_channel[9]  = (int16_t)((ptr[13] >> 3) | ((ptr[14] << 5) & 0x07FF));
	        msg_sbus->rx_channel[10] = (int16_t)((ptr[14] >> 6) | (ptr[15] << 2) |((ptr[16] << 10) & 0x07FF));
	        msg_sbus->rx_channel[11] = (int16_t)((ptr[16] >> 1) | ((ptr[17] << 7) & 0x07FF));
	        msg_sbus->rx_channel[12] = (int16_t)((ptr[17] >> 4) | ((ptr[18] << 4) & 0x07FF));
	        msg_sbus->rx_channel[13] = (int16_t)((ptr[18] >> 7) | (ptr[19] << 1) | ((ptr[20] << 9) & 0x07FF));
	        msg_sbus->rx_channel[14] = (int16_t)((ptr[20] >> 2) | ((ptr[21] << 6) & 0x07FF));
	        msg_sbus->rx_channel[15] = (int16_t)((ptr[21] >> 5) | ((ptr[22] << 3) & 0x07FF));
	        msg_sbus->rx_channel17 	= ptr[23] & MSG_SBUS_CH17_BIT_MASK;
	        msg_sbus->rx_channel18 	= ptr[23] & MSG_SBUS_CH18_BIT_MASK;
	        msg_sbus->frame_lost 	= ptr[23] & MSG_SBUS_FL_BIT_MASK;
	        msg_sbus->failsafe 		= ptr[23] & MSG_SBUS_FS_BIT_MASK;


		rx_recv_cnt[0]++;

		printf("cnt: %d\t ch[1]:%d\t ch[2]:%d\t ch[3]:%d\t ch[4]:%d\t ch[5]:%d\t ch[6]:%d\t ch[7]:%d\t ch[8]:%d\t FL:%d FS:%d\n",rx_recv_cnt[0], msg_sbus->rx_channel[0],msg_sbus->rx_channel[1],msg_sbus->rx_channel[2],msg_sbus->rx_channel[3],msg_sbus->rx_channel[4],msg_sbus->rx_channel[5],msg_sbus->rx_channel[6],msg_sbus->rx_channel[7], msg_sbus->frame_lost, msg_sbus->failsafe);
	}

	else
	{
		rx_err_cnt[0]++;
	}
//	printf("recv: %d\t err: %d\n", rx_recv_cnt[0], rx_err_cnt[0]);

}






