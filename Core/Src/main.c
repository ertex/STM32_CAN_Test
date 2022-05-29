/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint16_t disp1_number;
uint16_t disp2_number;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void display_digit(uint8_t number,uint8_t decimal){
	uint32_t odr;
	uint32_t newportdata;

	uint16_t digits[] =
		   {0b0101111100000000,
			0b0100010000000000,
			0b0011011100000000,
			0b0111010100000000,
			0b0110110000000000,
			0b0111100100000000,
			0b0110101100000000,
			0b0101010000000000,
			0b0111111100000000,
			0b0111110000000000};

	odr = GPIOB->ODR;
	newportdata = (uint32_t)((0x00FF & odr) | (digits[number]));
	if(decimal){
		newportdata = newportdata | 0x8000;
	}
	GPIOB->ODR = newportdata;


}

void write_portA(uint8_t data){
	uint16_t odr = GPIOA->ODR;
	GPIOA->ODR = (uint32_t)((0b1111111111000000 & odr) | data);
}

void display_number(uint16_t number,uint8_t divisor,uint8_t display){
	uint16_t digit = (number) % 10;

	GPIOA->ODR = (uint32_t)(((uint32_t)!0b00111111) & GPIOA->ODR);
	display_digit(digit,0);
	if(display == 1){
		HAL_GPIO_WritePin(GPIOA,DIG1_1_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA,DIG1_2_Pin,GPIO_PIN_SET);
	}

	digit = (number/10) % 10;

	GPIOA->ODR = (uint32_t)(((uint32_t)!0b00111111) & GPIOA->ODR);
	if(display == 1){
		display_digit(digit,1);
		HAL_GPIO_WritePin(GPIOA,DIG2_1_Pin,GPIO_PIN_SET);
	}else{
		display_digit(digit,0);
		HAL_GPIO_WritePin(GPIOA,DIG2_2_Pin,GPIO_PIN_SET);
	}

	digit = (number/100) % 10;

	GPIOA->ODR = (uint32_t)(((uint32_t)!0b00111111) & GPIOA->ODR);
	display_digit(digit,0);
	if(display == 1){
		HAL_GPIO_WritePin(GPIOA,DIG3_1_Pin,GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOA,DIG3_2_Pin,GPIO_PIN_SET);
	}
}

void HAL2_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);

  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
	    display_number(disp1_number,0,1);
	    display_number(disp2_number,0,0);
  }
}

void send_CAN(){

	//HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, &TxData);
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData) != HAL_OK)
					{

						/*Transmission request Error*/
						Error_Handler();
					}
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  */

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	HAL_GPIO_TogglePin(GPIOA,LED1_Pin);
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    /* Retrieve Rx messages from RX FIFO0 */
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
	  Error_Handler();
	}
	if(RxHeader.Identifier == 0x1B030141){
		// https://github.com/Cougar/HomeAutomation/blob/f2951eb0619c6122b6983275e1e698570eed4b5a/branches/modules/EmbeddedSoftware/AVR/module/sns_ds18x20/sns_ds18x20.c
		// - cel full celsius
		// - fractions of celsius in millicelsius*(10^-1)/625 (the 4 LS-Bits)

		//This code won't work with negative numbers
		uint16_t in_temp;
		uint16_t full_cel;
		uint16_t cel_frac_bits;
		uint16_t cel_frac;

		in_temp        = (RxData[1] << 8) | RxData[2];
		full_cel       = (in_temp & 0b1111111111000000) >> 6;
		cel_frac_bits  = (in_temp & 0b000000000000111111) >> 2;
		cel_frac       = cel_frac_bits*625; //fractional parts of the temperature, in integer form
		cel_frac       = cel_frac/1000;
		if(cel_frac > 9){
			disp2_number = 88;
		}else{
			disp2_number = 11;
		}

		disp1_number   = (full_cel * 10) + cel_frac;

	}
	if(RxHeader.Identifier == 0x14002101){
		uint16_t foo; //Possible loss with signed values
		foo = (RxData[1] << 8) | RxData[2];
	//	disp2_number = (foo & 0b1111111111000000) >> 6;
	}

  }
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
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */
  //uint8_t a = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//  a++;

	  //disp1_number = a;
	 // disp2_number = a;
	  HAL2_Delay(1000);
	  HAL_GPIO_TogglePin(GPIOA,LED2_Pin);
	 // if(!(a % 20)){
	//	  //send_CAN();
	//	  HAL_GPIO_TogglePin(GPIOA,LED2_Pin);
	 // }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
	FDCAN_FilterTypeDef sFilterConfig;
		/* Configure Rx filter */
		sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
		sFilterConfig.FilterIndex = 0;
		sFilterConfig.FilterType = FDCAN_FILTER_MASK;
		sFilterConfig.FilterID1 = 0x000;
		sFilterConfig.FilterID2 = 0x7FF;
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 16;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK){
  		Error_Handler();
  	}

  	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK){
  	  Error_Handler();
    }

  	if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK){
  		Error_Handler();
  	}

  	//INIT TXheader frame
	//TxHeader.Identifier = 0x1FFFFFFF;
	TxHeader.Identifier = 0b00101111111111111111111111111; //SNS_class
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.MessageMarker = 0;

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EXT_sig1_Pin|EXT_sig2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIG3_1_Pin|DIG2_1_Pin|DIG1_1_Pin|DIG1_2_Pin
                          |DIG2_2_Pin|LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_B_Pin|LED_F_Pin|LED_A_Pin|LED_G_Pin
                          |LED_C_Pin|LED_DP_Pin|LED_D_Pin|LED_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EXT_sig1_Pin EXT_sig2_Pin */
  GPIO_InitStruct.Pin = EXT_sig1_Pin|EXT_sig2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_sig3_Pin */
  GPIO_InitStruct.Pin = EXT_sig3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_sig3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIG3_1_Pin DIG2_1_Pin DIG1_1_Pin DIG1_2_Pin
                           DIG2_2_Pin LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = DIG3_1_Pin|DIG2_1_Pin|DIG1_1_Pin|DIG1_2_Pin
                          |DIG2_2_Pin|LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIG3_2_Pin */
  GPIO_InitStruct.Pin = DIG3_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIG3_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_B_Pin LED_F_Pin LED_A_Pin LED_G_Pin
                           LED_C_Pin LED_DP_Pin LED_D_Pin LED_E_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin|LED_F_Pin|LED_A_Pin|LED_G_Pin
                          |LED_C_Pin|LED_DP_Pin|LED_D_Pin|LED_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
