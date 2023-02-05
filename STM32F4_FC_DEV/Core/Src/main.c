/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ICM20602.h"
#include "Quaternion.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int _write(int file, char* p, int len)
{
	for(int i=0;i<len;i++)
	{
		LL_USART_TransmitData8(USART6, *(p+i));
		usDelay(100);	// 문자 1개 출력당 약 100us 소요, Float, int형 차이 없음
	}
	return len;
}

void UART3_GPS_TransmitDataString(char* p, int len)
{
	for(int i=0;i<len;i++)
	{
		LL_USART_TransmitData8(USART3, *(p+i));
		usDelay(10000);	// 문자 1개 출력당 약 100us 소요, Float, int형 차이 없음
	}

}


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t flag_INT_USART6;		// UART6 인터럽트 플래그 변수
extern uint8_t flag_INT_UART3_GPS;	// UART3 인터럽트 플래그 변수
extern uint8_t rxd, rxd_gps;		// UART 수신데이터 버퍼

extern unsigned int TimingDelay;
extern float sampleFreq;
extern float Roll;
extern float Pitch;
extern float Yaw;
extern float q[4];       // vector to hold quaternion


unsigned long  t1=0;
unsigned long  t2=0;
float gx_cal=0, gy_cal=0, gz_cal=0;	ax_cal=0; ay_cal=0;
float GYROX_RATE, GYROY_RATE, GYROZ_RATE, ACCX_RATE, ACCY_RATE, ACCZ_RATE;

int recv_pack_gps=0;
unsigned char buf_gps[100];

unsigned char M8N_CFG_PRT[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x54};
unsigned char M8N_CFG_MSG[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x29, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x3B, 0xFE};
unsigned char M8N_CFG_RAT[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
unsigned char M8N_CFG_CFG[21] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_InitTick(168000000, 1000000U);		//	Clock을 1us단위로 조정, 1ms함수 사용할 수 없음
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  LL_TIM_EnableCounter(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);

  TIM3->PSC = 2000;
  usDelay(100000);
  TIM3->PSC = 1500;
  usDelay(100000);
  TIM3->PSC = 1000;
  usDelay(100000);



  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1);

  LL_USART_EnableIT_RXNE(USART6);	// UART6 인터럽트 활성화
  LL_USART_EnableIT_RXNE(USART3);	// UART3 인터럽트 활성화
  ICM20602_Initialization();

  q[0] = 1.0f;
  q[1] = 0.0f;
  q[2] = 0.0f;
  q[3] = 0.0f;



  // GPS UBX 모드 초기화: 동작 확인 안됨
/*  UART3_GPS_TransmitDataString(&M8N_CFG_PRT[0], 28);
  usDelay(200000);
  UART3_GPS_TransmitDataString(&M8N_CFG_MSG[0], 16);
  usDelay(200000);
  UART3_GPS_TransmitDataString(&M8N_CFG_RAT[0], 14);
  usDelay(200000);
  UART3_GPS_TransmitDataString(&M8N_CFG_CFG[0], 21);
  usDelay(200000);
*/

//  memset(&buf_gps, 0, 100);

  // 시간측정을 위한 레지스터 초기화 값
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  //	  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
  //	  usDelay(1000000);
  //	  LL_USART_TransmitData8(USART6,'B');

	  if(flag_INT_USART6 == 1){
		  flag_INT_USART6 =0;
		  LL_USART_TransmitData8(USART3,rxd); // 터미널에서 수신된 데이터를 GPS로 전달
//		  LL_USART_TransmitData8(USART6,rxd); // 호스트로부터 수신한 데이터를 그대로 다시 보냄
//		  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);

	  }

	  if(flag_INT_UART3_GPS == 1){
		  flag_INT_UART3_GPS =0;
		  LL_USART_TransmitData8(USART6,rxd_gps); // GPS에서 수신된 데이터를 터미널로 전달

	  }

	  if(ICM20602_DataReady() == 1)
	  {

		  t2 = DWT->CYCCNT;
		  sampleFreq = (1000000.0f /(((float)(t2-t1))/CLOCK_PER_USEC)); // set integration time by time elapsed since last filter update
		  t1 = t2;

		  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);		// 0.5us
		  ICM20602_Get6AxisRawData(&ICM20602.acc_x_raw, &ICM20602.gyro_x_raw);	//	39.11us
		  //		  GYROX_RATE = (ICM20602.gyro_x_raw - gx_cal) * 0.06103515625 * 0.017453289;
		  GYROX_RATE = (ICM20602.gyro_x_raw - gx_cal) * 0.06103515625 * 0.017453289;
		  GYROY_RATE = (ICM20602.gyro_y_raw - gy_cal) * 0.06103515625 * 0.017453289;
		  GYROZ_RATE = (ICM20602.gyro_z_raw - gz_cal) * 0.06103515625 * 0.017453289;

		  ACCX_RATE = (ICM20602.acc_x_raw - ax_cal) * 0.00048828125;
		  ACCY_RATE = (ICM20602.acc_y_raw - ay_cal) * 0.00048828125;
		  ACCZ_RATE = (ICM20602.acc_z_raw) * 0.00048828125;

//		  MadgwickQuaternionUpdate(&ACCX_RATE,&ACCY_RATE,&ACCZ_RATE,&GYROX_RATE,&GYROY_RATE,&GYROZ_RATE);	//57us
		  MahonyAHRSupdateIMU(&GYROX_RATE,&GYROY_RATE,&GYROZ_RATE, &ACCX_RATE,&ACCY_RATE,&ACCZ_RATE);		//42us
		  Quaternion_Update(&q);	//10us

//		  printf("%d %d %d\n", (int)(Roll), (int)(Pitch), (int)(Yaw));
//		  printf("%d %d %d\n", (int)(q[0]*100), (int)(q[1]*100),(int)(q[2]*100));
//		  printf("%.2f\n",(sampleFreq));
//		  printf("%.d %.d %.d\n", ICM20602.gyro_x_raw, ICM20602.gyro_y_raw, ICM20602.gyro_z_raw);
//		  printf("%.1f %.1f %.1f\n", ACCX_RATE, ACCY_RATE, ACCZ_RATE);

		  // overall 230us time consumption
		  // Loop 수행시간 계산시 소수점 세자리 출력(약 400u) 더해주어야 함

//		  t2 = DWT->CYCCNT;
//		  printf("%.0f\n",((float)(t2-t1))/CLOCK_PER_USEC);
//		  t1 = DWT->CYCCNT;

	  }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_5)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 168, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(168000000);
  LL_SetSystemCoreClock(168000000);
}

/* USER CODE BEGIN 4 */
void usDelay(unsigned int nTime)
{
	__IO unsigned int  tmp = SysTick->CTRL;
	((void)tmp);

	SysTick->VAL = 0;

	SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; // clock source

	TimingDelay = nTime-1;

	while(TimingDelay);

	SysTick->CTRL = 0;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
