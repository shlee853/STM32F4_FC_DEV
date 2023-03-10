

#ifndef INC_GPS_H_
#define INC_GPS_H_

	#include "main.h"

	#define MSG_UBX_SOF1 			0xB5
	#define MSG_UBX_SOF2			0x62
	#define CLASS_ID_UBX 			0x01
	#define MSG_ID_UBX_NAV_POSLLH	0x02
	#define MSG_ID_UBX_NAV_SOL 		0x06
	#define MSG_LENGTH_NAV_POSLLH 	28+8
	#define MSG_LENGTH_NAV_SOL 		52+8
	#define LATITUDE_OFFSET 		33
	#define LONGITUDE_OFFSET 		124


	typedef struct{
		USART_TypeDef* UART;
		DMA_TypeDef* DMA;
		uint8_t gps_raw_buf[MSG_LENGTH_NAV_SOL];
	}GPS_RAW_MESSAGE;


	typedef struct{
		int32_t lon;
		int32_t lat;

		int32_t longitude_deg,longitude_min;
		float longitude_sec;

		int32_t latitude_deg,latitude_min;
		float latitude_sec;

		float sec_lon;
		float sec_lat;
	}GPS_DATA;


	typedef struct{

		uint32_t iTOW;		// GPS time(ms)
		int32_t lon;		// Longitude(deg)	scaling: e-7
		int32_t lat;		// Latitude(deg)	scaling: e-7
		int32_t h;		// Height(mm)
		int32_t hMSL;		// Height above mean sea level(mm)
		uint32_t hAcc;		// Horizontal Accuracy Estimate(mm)
		uint32_t vAcc;		// Vertical Accuracy Estimate(mm)

	}MSG_NAV;




	void GPS_DMA_init(GPS_RAW_MESSAGE* gps_raw_message, USART_TypeDef* UART,DMA_TypeDef* DMA,uint32_t DMA_STREAM);
	void GPS_Parsing(GPS_RAW_MESSAGE* message, MSG_NAV* gps_data, int* recv_cnt, int* err_cnt);



#endif /* INC_GPS_H_ */
