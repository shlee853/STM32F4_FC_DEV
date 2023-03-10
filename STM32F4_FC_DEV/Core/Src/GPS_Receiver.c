
#include "GPS_Receiver.h"



int recv_cnt;
int err_cnt;

void GPS_DMA_init(GPS_RAW_MESSAGE* gps_raw_message, USART_TypeDef* UART,DMA_TypeDef* DMA,uint32_t DMA_STREAM)
{


	//DMA, INTERRUPT SETTINGS
	LL_DMA_SetMemoryAddress(DMA,DMA_STREAM,(uint32_t)(gps_raw_message->gps_raw_buf));
	LL_DMA_SetPeriphAddress(DMA,DMA_STREAM,LL_USART_DMA_GetRegAddr(UART));
	LL_DMA_SetDataLength(DMA,DMA_STREAM,MSG_LENGTH_NAV_SOL);

	//  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_1);
    LL_DMA_EnableIT_TC(DMA, DMA_STREAM);
	LL_USART_EnableDMAReq_RX(UART);

}



void GPS_Parsing(GPS_RAW_MESSAGE* message, MSG_NAV* msg_nav, int* recv_cnt, int* err_cnt)
{
	uint8_t* ptr,*gps_ptr = message->gps_raw_buf;
	int32_t temp;
	unsigned short msg_length = 0, checksum = 0;
	unsigned char classID = 0, messageID = 0;
	if(gps_ptr[0]==MSG_UBX_SOF1 && gps_ptr[1]==MSG_UBX_SOF2){

		ptr 		= gps_ptr + 2;
		classID 	= *ptr++;
		messageID 	= *ptr++;

		if(messageID==MSG_ID_UBX_NAV_POSLLH) {
			msg_length 	= (ptr[1] << 8) + (ptr[0]);
			ptr = ptr +2;
			memcpy(msg_nav, ptr, msg_length);

//			printf("time:[%d]ms\t lon: [%d]deg\t lat: [%d]deg\t h: [%d]mm\t hMSL: [%d]mm\t hAcc: [%d]mm\t vAcc: [%d]mm\n",msg_nav->iTOW, msg_nav->lon,msg_nav->lat, msg_nav->h, msg_nav->hMSL, msg_nav->hAcc, msg_nav->vAcc);

		}
		else if(messageID==MSG_ID_UBX_NAV_SOL) {
			msg_length 	= (ptr[1] << 8) + (ptr[0]);
			ptr = ptr +2;
			msg_nav = msg_nav + MSG_LENGTH_NAV_POSLLH - 8;
			memcpy(msg_nav, ptr, msg_length);
		}

		recv_cnt[0]++;

	}

	else
	{
		err_cnt[0]++;
	}
//	printf("recv: %d\t err: %d\n", recv_cnt[0], err_cnt[0]);

}



