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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Configure.h"
#include "ICM20602.h"
#include "Quaternion.h"
#include "GPS_Receiver.h"
#include "SBUS_Receiver.h"
#include "W25Qxx_Flash.h"
//#include "cli.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t flag_INT_UART1_RX, flag_INT_UART1_RX_DONE;	// SBUS 수신기 인터럽트 플래그 변수
extern uint8_t flag_INT_USART6,flag_INT_UART6_RX_DONE;		// UART6 인터럽트 플래그 변수
extern uint8_t flag_INT_UART3_GPS;	// UART3 인터럽트 플래그 변수
extern uint8_t flag_INT_UART4_RX;	// S.Port 수신기 인터럽트 플래그 변수
extern uint8_t flag_DMA1_DONE, flag_DMA2_DONE;		// DMA1_STREAM1 DMA완료 플래그

extern uint8_t rxd, rxd_gps;		// UART 수신데이터 버퍼
extern uint8_t rx_buf[256];		// Rx 수신데이터 버퍼
uint8_t cnt1=0, cnt2 = 0, radio_safe=0;

//Flash
uint8_t flash_data[8]= {0x01,0x02,0x03,0x04, 0xAA,0x55,0xA5,0x5A};
uint8_t read_flash=0;
uint8_t read_flash_arr[256], write_flash_arr[256];
uint8_t flash_byte = 0x05, read_byte=0;

uint16_t adcVal;
float BatVol=0;


extern unsigned int TimingDelay;

unsigned long  t1=0;
unsigned long  t2=0;

float sampleFreq;

extern int recv_cnt, rx_recv_cnt;
extern int err_cnt, rx_err_cnt;


GPS_RAW_MESSAGE raw_gps;
SBUS_RAW_MESSAGE raw_rx;
MSG_NAV 		msg_nav;
MSG_SBUS 		msg_sbus;
//unsigned char M8N_CFG_PRT[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8B, 0x54};
//unsigned char M8N_CFG_MSG[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x29, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x3B, 0xFE};
//unsigned char M8N_CFG_RAT[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
//unsigned char M8N_CFG_CFG[21] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0x31, 0xBF};





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
  HAL_Init();

  /* USER CODE BEGIN Init */
  memset(&read_flash_arr,0,256);
  for(int i=0; i<256;i++){
	  write_flash_arr[i]=i;
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_InitTick(168000000, 1000000U);		//	Clock을 1us단위로 조정, 1ms함수 사용할 수 없음
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_SPI3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  W25QXX_SPI_Initialization();
//  W25qxx_EraseChip();
//  W25qxx_EraseSector(0);
//  W25qxx_IsEmptyBlock(0, 0, 256);

//  W25qxx_WriteByte(0x05,5);
//  W25qxx_ReadBytes(read_flash_arr, 0, 10);

//  W25qxx_WritePage(write_flash_arr, 0, 0, 256);
//  W25qxx_ReadPage(read_flash_arr, 0, 0,  256);
//  W25qxx_ReadBytes(read_flash_arr, 0, 256);
//  W25qxx_ReadBytes(read_flash_arr, 5, 5);


  // Buzzer
/*  LL_TIM_EnableCounter(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);

  TIM3->PSC = 2000;
  usDelay(100000);
  TIM3->PSC = 1500;
  usDelay(100000);
  TIM3->PSC = 1000;
  usDelay(100000);

  LL_TIM_CC_DisableChannel(TIM3, LL_TIM_CHANNEL_CH1);
*/
  // ESC PWM
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);	// M1
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH4); 	// M2
  LL_TIM_EnableCounter(TIM5);
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH4);	// M3
  LL_TIM_CC_EnableChannel(TIM5, LL_TIM_CHANNEL_CH3);	// M4

  // ESC Calibration
/*  TIM3->CCR3 = MAX_TIM_PWM;
  TIM3->CCR4 = MAX_TIM_PWM;
  TIM5->CCR4 = MAX_TIM_PWM;
  TIM5->CCR3 = MAX_TIM_PWM;
  usDelay(7000000);	// 7sec

  TIM3->CCR3 = MIN_TIM_PWM;
  TIM3->CCR4 = MIN_TIM_PWM;
  TIM5->CCR4 = MIN_TIM_PWM;
  TIM5->CCR3 = MIN_TIM_PWM;
  usDelay(8000000);	// 7sec
*/

  ICM20602_Initialization();

  // Batery ADC Check
  HAL_ADC_Start_DMA(&hadc1, &adcVal, 1);


  // 시간측정을 위한 레지스터 초기화 값
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // UART3 GPS 수신기 DMA 및 인터럽트 설정
  GPS_DMA_init(&raw_gps, USART3, DMA1, LL_DMA_STREAM_1);
  LL_DMA_EnableStream(DMA1,LL_DMA_STREAM_1);
  LL_USART_EnableIT_IDLE(USART3);

  // UART1 SBUS 수신기 인터럽트 설정
  LL_USART_EnableIT_IDLE(USART1);
  LL_USART_EnableIT_RXNE(USART1);

  // UART6 문자열 인터페이스 DMA 및  인터럽트 설정
//  USART_DMA_Transmit_INIT(USART6, DMA2, LL_DMA_STREAM_6);
  LL_USART_EnableIT_RXNE(USART6);	// UART6 인터럽트 활성화
//  LL_USART_EnableIT_IDLE(USART6);
//  LL_USART_EnableIT_TC(USART6);

  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);

  CliInit(&huart6);
  FLASH_If_Init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  //	  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
  //	  usDelay(1000000);
  //	  LL_USART_TransmitData8(USART6,'B');


	  if(flag_INT_UART1_RX==1){
		  raw_rx.rx_buf[cnt1++] = LL_USART_ReceiveData8(USART1);
          flag_INT_UART1_RX =0;
	  }

	  if(flag_INT_UART1_RX_DONE == 1)
	  {
		  SBUS_Parsing(&raw_rx, &msg_sbus, &rx_recv_cnt, &rx_err_cnt);
          cnt1=0;
		  flag_INT_UART1_RX_DONE = 0;
	  }

	  if(flag_INT_UART4_RX==1){
		  flag_INT_UART4_RX = 0;
	  }


	  if(flag_INT_USART6 == 1){
		  flag_INT_USART6 =0;

		  CliDo(&huart6);


//		  LL_USART_TransmitData8(USART3,rxd); // 터미널에서 수신된 데이터를 GPS로 전달
//		  LL_USART_TransmitData8(USART6,rxd); // 호스트로부터 수신한 데이터를 그대로 다시 보냄
//		  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
	  }

	  // UART3에서 한 프레임 GPS 데이터 수신완료에 인터럽트 발생, GPS데이터가 수신될 때마다 DMA는 데이터카운트를 하나씩 감소하면서 0이 될 때까지 전송
	  if(flag_INT_UART3_GPS == 1){

//		  cnt=(uint8_t)(MSG_LENGTH_NAV_SOL-LL_DMA_GetDataLength(DMA1,LL_DMA_STREAM_1));
		  LL_DMA_DisableStream(DMA1,LL_DMA_STREAM_1);
		  LL_DMA_ClearFlag_TC1(DMA1);
//		  cnt = 0;
		  flag_INT_UART3_GPS =0;

	  }

	  // DMA 데이터카운트가 0이되면 인터럽트 발생, 데이터를 버퍼로 수신 완료
	  if(flag_DMA1_DONE == 1)
	  {
//		  cnt=(uint8_t)(MSG_LENGTH_NAV_SOL-LL_DMA_GetDataLength(DMA1,LL_DMA_STREAM_1));

		  // Batery Checker
		  BatVol = adcVal*ADC_BAT_COEFF;
//		  printf("Battery Check: ADC[%d], Voltage[%f]\n", adcVal, BatVol);

		  // GPS Parsing
		  GPS_Parsing(&raw_gps, &msg_nav, &recv_cnt, &err_cnt);
		  LL_DMA_EnableStream(DMA1,LL_DMA_STREAM_1);
		  flag_DMA1_DONE = 0;
	  }


	  if(ICM20602_DataReady() == 1)
	  {

		  t2 = DWT->CYCCNT;
//		  sampleFreq = (1000000.0f /(((float)(t2-t1))/CLOCK_PER_USEC)); // set integration time by time elapsed since last filter update
		  t1 = t2;
//		  printf("%.2f\n",(sampleFreq));
		  GetRPY(&sampleFreq);
		  // overall 230us time consumption
		  // Loop 수행시간 계산시 소수점 세자리 출력(약 400u) 더해주어야 함

	  }


	  // Failsafe
	  if(msg_sbus.failsafe==MSG_SBUS_FS_BIT_MASK) {
		  TIM3->CCR3 = MIN_TIM_PWM;
		  TIM3->CCR4 = MIN_TIM_PWM;
		  TIM5->CCR4 = MIN_TIM_PWM;
		  TIM5->CCR3 = MIN_TIM_PWM;
	  }

	  // Radio Control
	  else	  {

		  if(radio_safe==0) {
			  if((msg_sbus.rx_channel[0] < MIN_RADIO_CH1_PWM + 5)&&(msg_sbus.rx_channel[0] > 0)) {
				   LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
				   usDelay(500000);
				   LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
				   usDelay(500000);
				   LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
				   usDelay(500000);
				   LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
				   usDelay(500000);
				   LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
				   usDelay(500000);
				   LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_5);
				   usDelay(500000);
				   radio_safe = 1;
			  }
		  }

		  else {
			  TIM3->CCR3 = MIN_TIM_PWM + RANGE_TIM_PWM/RANGE_RADIO_CH1_PWM*(msg_sbus.rx_channel[0]-MIN_RADIO_CH1_PWM);
			  TIM3->CCR4 = MIN_TIM_PWM + RANGE_TIM_PWM/RANGE_RADIO_CH1_PWM*(msg_sbus.rx_channel[0]-MIN_RADIO_CH1_PWM);
			  TIM5->CCR4 = MIN_TIM_PWM + RANGE_TIM_PWM/RANGE_RADIO_CH1_PWM*(msg_sbus.rx_channel[0]-MIN_RADIO_CH1_PWM);
			  TIM5->CCR3 = MIN_TIM_PWM + RANGE_TIM_PWM/RANGE_RADIO_CH1_PWM*(msg_sbus.rx_channel[0]-MIN_RADIO_CH1_PWM);
		  }

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
  LL_SetSystemCoreClock(168000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
