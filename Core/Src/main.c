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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../lmic/lmic.h"
#include "../../stm32/debug.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Max size array
#define SIZEARRAY 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int a =0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// application router ID (LSBF)  < ------- IMPORTANT
// application router ID (LSBF)
static const uint8_t APPEUI[8]  = {}; //{ 0xC1 , 0x67 , 0x01 , 0xD0 , 0x7E , 0xD5 , 0xB3 , 0x70};

// unique device ID (LSBF)
static const uint8_t DEVEUI[8]  ={ 0x70, 0xB3, 0xD5, 0x7E, 0xD8, 0x00, 0x21, 0x35 }; //{ 0x5b , 0xfe , 0xA1 , 0x59 , 0x2f , 0xf1 , 0x56 , 0x00 };

// device-specific AES key (derived from device EUI)
static const uint8_t DEVKEY[16] = {}; //{ 0x02, 0x25, 0xC3, 0x06, 0x01, 0xAF, 0xA9, 0xEC, 0x05, 0x74, 0xA2, 0x66, 0xA5, 0xAA, 0xE3, 0x18 };
// network.
static const uint8_t NWKSKEY[16] = { 0x5F, 0xCC, 0x77, 0xA8, 0x6A, 0xCB, 0x59, 0x7D, 0x5B, 0xD9, 0x80, 0x23, 0x13, 0x45, 0x94, 0x73 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const uint8_t APPSKEY[16] = { 0xAC, 0x06, 0x47, 0xBB, 0x19, 0xE8, 0x25, 0x1D, 0xF3, 0xB5, 0x70, 0xC6, 0x0C, 0xB0, 0x9A, 0x54 };

// LoRaWAN end-device address (DevAddr)
static const uint32_t DEVADDR = 0x27FD1B80; // <-- Change this address for every node!

//   ABP
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void deleteBuffer(char* data)
{
  uint8_t len = strlen(data);
  for(uint8_t i = 0; i < len; i++)
  {
		data[i] = 0;
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void os_getArtEui (uint8_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (uint8_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (uint8_t* buf) {
    memcpy(buf, DEVKEY, 16);
}

#define TX_INTERVAL 10

#define TX_TIMEOUT 60
uint8_t mydata[] = "Em chan anh roi a?";
//static uint8_t data[];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//void do_send(osjob_t* j){
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
//  // Check if there is not a current TX/RX job running
//  if (LMIC.opmode & OP_TXRXPEND) {
//    debug_str("OP_TXRXPEND, not sending\n");
//  } else {
//    // Prepare upstream data transmission at the next possible time.
//    uint8_t data[] = "hello world"; // Replace sensor data with this string
//    LMIC_setTxData2(1, data, sizeof(data), 0);
//
//    char buf[100];
//    sprintf(buf, "Packet queued txcnt %d\n", LMIC.txCnt);
//    debug_str(buf);
//  }
//  // Next TX is scheduled after TX_COMPLETE event.
//}

void do_send(osjob_t* j){
	ostime_t tm = os_getTime();
		printf("Call %s %lums\n", __func__, (unsigned long)osticks2ms(tm));

		if (!(LMIC.opmode & OP_TXRXPEND)) {
			//unsigned char txt[5] = "LoRa";
			LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
		}
		os_setTimedCallback(j, tm+sec2osticks(TX_TIMEOUT), do_send);
}
void onEvent (ev_t ev) {
    debug_event(ev);
    debug_str("\n");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            debug_str("EV_SCAN_TIMEOUT\n");
            break;
        case EV_BEACON_FOUND:
            debug_str("EV_BEACON_FOUND\n");
            break;
        case EV_BEACON_MISSED:
            debug_str("EV_BEACON_MISSED\n");
            break;
        case EV_BEACON_TRACKED:
            debug_str("EV_BEACON_TRACKED\n");
            break;
        case EV_JOINING:
            debug_str("EV_JOINING\n");
            break;
        case EV_JOINED:
            debug_str("EV_JOINED\n");
            break;
        case EV_RFU1:
            debug_str("EV_RFU1\n");
            break;
        case EV_JOIN_FAILED:
            debug_str("EV_JOIN_FAILED\n");
            break;
        case EV_REJOIN_FAILED:
            debug_str("EV_REJOIN_FAILED\n");
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_TXCOMPLETE:
            debug_str("EV_TXCOMPLETE (includes waiting for RX windows)\n");
            if (LMIC.txrxFlags & TXRX_ACK)
              debug_str("Received ack\n");
            if (LMIC.dataLen) {
            	a=1;
              debug_str("Received \n");
              debug_int(LMIC.dataLen);
              debug_str(" bytes of payload\n");
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            debug_str("EV_LOST_TSYNC\n");
            break;
        case EV_RESET:
            debug_str("EV_RESET\n");
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            debug_str("EV_RXCOMPLETE\n");
            break;
        case EV_LINK_DEAD:
            debug_str("EV_LINK_DEAD\n");
            break;
        case EV_LINK_ALIVE:
            debug_str("EV_LINK_ALIVE\n");
            break;
         default:
            debug_str("Unknown event\n");
            break;
    }
}

void initfunc(osjob_t* j)
{
	   // Reset the MAC state. Session and pending data transfers will be discarded.
	   LMIC_reset();
	   LMIC_startJoining();
	   // Set static session parameters. Instead of dynamically establishing a session
	   // by joining the network, precomputed session parameters are be provided.
	   // On AVR, these values are stored in flash and only copied to RAM
	   // once. Copy them to a temporary buffer here, LMIC_setSession will
	   // copy them into a buffer of its own again.
	    uint8_t appskey[sizeof(APPSKEY)];
	    uint8_t nwkskey[sizeof(NWKSKEY)];
	    memcpy(appskey, APPSKEY, sizeof(APPSKEY));
	    memcpy(nwkskey, NWKSKEY, sizeof(NWKSKEY));
	    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);


	  #if defined(CFG_eu868)
	   // Set up the channels used by the Things Network, which corresponds
	   // to the defaults of most gateways. Without this, only three base
	   // channels from the LoRaWAN specification are used, which certainly
	   // works, so it is good for debugging, but can overload those
	   // frequencies, so be sure to configure the full frequency range of
	   // your network here (unless your network autoconfigures them).
	   // Setting up channels should happen after LMIC_setSession, as that
	   // configures the minimal channel set.
	   // NA-US channels 0-71 are configured automatically
	   LMIC_setupChannel(0, 903900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(1, 904100000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
	   LMIC_setupChannel(2, 904300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(3, 904500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(4, 904700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(5, 904900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(6, 905100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(7, 905300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
	   LMIC_setupChannel(8, 904600000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
	   // TTN defines an additional channel at 869.525Mhz using SF9 for class B
	   // devices' ping slots. LMIC does not have an easy way to define set this
	   // frequency and support for class B is spotty and untested, so this
	   // frequency is not configured here.
	   #elif defined(CFG_us915)
	   // NA-US channels 0-71 are configured automatically
	   // but only one group of 8 should (a subband) should be active
	   // TTN recommends the second sub band, 1 in a zero based count.
	   // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
	   LMIC_selectSubBand(1);
	  #endif

	   // Disable link check validation
	   LMIC_setLinkCheckMode(0);

	   // TTN uses SF9 for its RX2 window.
	   LMIC.dn2Dr = DR_SF9;

	   // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	   LMIC_setDrTxpow(DR_SF9,14);
	   os_setTimedCallback(&sendjob, os_getTime(), do_send);
	   // Start job

	  // time= HAL_GetTick();
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);    // <-----------  change to your setup
  HAL_TIM_Base_Start_IT(&htim7);    // <-----------  change to your setup
   __HAL_SPI_ENABLE(&hspi2);      	// <-----------  change to your setup
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
osjob_t initjob;
os_init();
initfunc(&initjob);
do_send(&sendjob);
  while (1)
  {
    /* USER CODE END WHILE */
	  os_runloop_once();
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 244-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 30000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1221-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, lora_NSS_PIN_Pin|lora_Reset_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : lora_DIO1_PIN_Pin lora_DIO2_PIN_Pin lora_DIO0_PIN_Pin */
  GPIO_InitStruct.Pin = lora_DIO1_PIN_Pin|lora_DIO2_PIN_Pin|lora_DIO0_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : lora_NSS_PIN_Pin lora_Reset_PIN_Pin */
  GPIO_InitStruct.Pin = lora_NSS_PIN_Pin|lora_Reset_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
