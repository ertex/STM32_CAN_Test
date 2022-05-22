Core lines of code to get the STM32 to work with 250KBaud (20MHz to CAN controller)

static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
	FDCAN_FilterTypeDef sFilterConfig;
		/* Configure Rx filter */
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
		sFilterConfig.FilterIndex = 0;
		sFilterConfig.FilterType = FDCAN_FILTER_MASK;
		sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
		sFilterConfig.FilterID1 = 0x701;
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

  	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,FDCAN_IT_TX_COMPLETE) != HAL_OK){
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