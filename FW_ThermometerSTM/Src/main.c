/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "nrf24.h"
#include <stdio.h>
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
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define SHT21_ADDRESS 0x80  //I2C address for the sensor
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5

int16_t GetSHT21measurement(uint8_t command)
{
	uint8_t i2cmsg[3], crc;
	uint32_t	t1;
	int16_t result;
	static int transmission_counter;
	HAL_StatusTypeDef	i2cStatus = HAL_ERROR;

	//Measure
	i2cmsg[0]=command;
	i2cStatus = HAL_I2C_Master_Transmit(&hi2c1, SHT21_ADDRESS, i2cmsg, 1, 10);
	//Temperature conversion takes up to 85ms, while humidity 29ms.
	t1 = HAL_GetTick();
	transmission_counter=0;
	i2cStatus = HAL_ERROR;
	while (i2cStatus != HAL_OK)
	{
		HAL_Delay(5);
		i2cStatus = HAL_I2C_Master_Receive(&hi2c1, SHT21_ADDRESS, i2cmsg, 3, 10);
		if (i2cStatus == HAL_OK)
		{
			transmission_counter++;
			MX_CRC_Init();
			crc = HAL_CRC_Calculate (&hcrc, (uint32_t*)i2cmsg, 2);
			HAL_CRC_DeInit(&hcrc);
			if (crc == i2cmsg[2])	// if crc OK, continue, else repeat transmission
			{
				break;
			}
			else
			{
				i2cStatus = HAL_ERROR;
			}
		}
		if (HAL_GetTick()-t1>150) break;
	}
	result=((int)i2cmsg[0]<<8) | (i2cmsg[1] & (~0x03));
	
	if (i2cStatus != HAL_OK) result = 0xFFFF;
	
	return result;
}

uint16_t i16Temp, i16Hum;
void ReadSHT21(void)
{
	//Turn on power supply for the SHT11 sensor
	HAL_GPIO_WritePin(VCC_SHT_GPIO_Port,VCC_SHT_Pin,GPIO_PIN_SET);
  MX_I2C1_Init();
	HAL_Delay(20);	//Startup takes a minimum of 15ms

	i16Temp = GetSHT21measurement(TRIGGER_TEMP_MEASURE_NOHOLD);
	i16Hum = GetSHT21measurement(TRIGGER_HUMD_MEASURE_NOHOLD);
	//Measure humidity

	//Turn off I2C and SHT power
	HAL_I2C_DeInit(&hi2c1);
	//HAL_GPIO_WritePin(VCC_SHT_GPIO_Port,VCC_SHT_Pin,GPIO_PIN_RESET);
	HAL_GPIO_DeInit(VCC_SHT_GPIO_Port, VCC_SHT_Pin);	//Should go into Analog input mode
}

uint8_t Vdd;
void MeasureVDD(void)
{
	Vdd=19;
	
	PWR_PVDTypeDef	PVDcfg;
	PVDcfg.Mode=PWR_PVD_MODE_NORMAL;	//No event, no interrupt?
	PVDcfg.PVDLevel=PWR_PVDLEVEL_0;	//0=1.9V, 1=2.1V, 2=2.3V, ... 6=3.1V, 7=ext
	
	HAL_PWR_ConfigPVD(&PVDcfg);
	HAL_PWR_EnablePVD();
	
	if (!__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) )
	{
		PVDcfg.PVDLevel=PWR_PVDLEVEL_1;	//0=1.9V, 1=2.1V, 2=2.3V, ... 6=3.1V, 7=ext
		HAL_PWR_ConfigPVD(&PVDcfg);
		Vdd+=2;
		if (!__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) )
		{
			PVDcfg.PVDLevel=PWR_PVDLEVEL_2;	//0=1.9V, 1=2.1V, 2=2.3V, ... 6=3.1V, 7=ext
			HAL_PWR_ConfigPVD(&PVDcfg);
			Vdd+=2;
			if (!__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) )
			{
				PVDcfg.PVDLevel=PWR_PVDLEVEL_3;	//0=1.9V, 1=2.1V, 2=2.3V, ... 6=3.1V, 7=ext
				HAL_PWR_ConfigPVD(&PVDcfg);
				Vdd+=2;
				if (!__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) )
				{
					PVDcfg.PVDLevel=PWR_PVDLEVEL_4;	//0=1.9V, 1=2.1V, 2=2.3V, ... 6=3.1V, 7=ext
					HAL_PWR_ConfigPVD(&PVDcfg);
					Vdd+=2;
					if (!__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) )
					{
						PVDcfg.PVDLevel=PWR_PVDLEVEL_5;	//0=1.9V, 1=2.1V, 2=2.3V, ... 6=3.1V, 7=ext
						HAL_PWR_ConfigPVD(&PVDcfg);
						Vdd+=2;
						if (!__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) )
						{
							PVDcfg.PVDLevel=PWR_PVDLEVEL_6;	//0=1.9V, 1=2.1V, 2=2.3V, ... 6=3.1V, 7=ext
							HAL_PWR_ConfigPVD(&PVDcfg);
							Vdd+=2;
						}
					}
				}
			}
		}
	}
	HAL_PWR_DisablePVD();
}

uint8_t TransmitPacket(uint8_t ID, uint16_t data, uint8_t rtrCount)
{
	uint8_t transmissionStatus;
	uint8_t data_array[5];
	uint32_t t1;
	
	data_array[0] = ID;
	data_array[1] = data>>8;
	data_array[2] = data;
	data_array[3] = Vdd;                                    
	data_array[4] = rtrCount;                                    

	nrf24_send(data_array);        
	t1 = HAL_GetTick();
	while(nrf24_isSending())
	{
		if ( (HAL_GetTick()-t1) > 200 ) break;	//Needed if NRF disconnected 
	}		
	transmissionStatus = nrf24_lastMessageStatus();
	rtrCount = nrf24_retransmissionCount();

	if(transmissionStatus == NRF24_TRANSMISSON_OK)
	{
		//All OK. Nothing to do.
	}
	else if(transmissionStatus == NRF24_MESSAGE_LOST)
	{                    
		//TODO: Is there anything I can do in case of error
	}
	else
	{
		//TODO: check if there can be any other kind of status
	}
			
	return rtrCount;
}

void TransmitData(void)
{
	static uint8_t retransmissionCount=0;	//always send retransmission count of the previous transmission
	uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};

	MX_SPI1_Init();

	nrf24_init();
	nrf24_config(2,4); 	// Channel #2 , payload length: 4 
	nrf24_tx_address(tx_address);// Set the device addresses
	nrf24_rx_address(rx_address);    

	retransmissionCount = TransmitPacket(SENSOR_ID(STYPE_T), i16Temp, retransmissionCount);
	retransmissionCount = TransmitPacket(SENSOR_ID(STYPE_H), i16Hum, retransmissionCount);
	
	HAL_SPI_DeInit(&hspi1);
	HAL_GPIO_DeInit(CE_GPIO_Port, CE_Pin);	//Should go into Analog input mode
	HAL_GPIO_DeInit(nCS_GPIO_Port, nCS_Pin);	//Should go into Analog input mode
	HAL_GPIO_DeInit(IRQ_GPIO_Port, IRQ_Pin);	//Should go into Analog input mode

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

	#if 0		//Disable standard startup - turn on each thing when needed, then off again
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	
	#endif
  
	HAL_GPIO_DeInit(PMODE_GPIO_Port, PMODE_Pin);	//Should go into Analog input mode
	HAL_GPIO_DeInit(VCC_DS_GPIO_Port, VCC_DS_Pin);	//Should go into Analog input mode
	HAL_GPIO_DeInit(DS_DATA_GPIO_Port, DS_DATA_Pin);	//Should go into Analog input mode

	MX_RTC_Init();
	HAL_PWREx_EnableUltraLowPower(); // Ultra low power mode
	HAL_PWREx_EnableFastWakeUp(); // Fast wake-up for ultra low power mode
	__HAL_RTC_WAKEUPTIMER_EXTI_DISABLE_IT();
	__HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_EVENT();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//static float fTemp,fhum;
  while (1)
  {
		MX_GPIO_Init();
		ReadSHT21();
		MeasureVDD();
//		fTemp = -46.85 + 175.72 / 65536.0 * i16Temp;
//		fhum = -6.0 + 125.0 / 65536.0 * i16Hum;
		TransmitData();

		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON,PWR_STOPENTRY_WFE);
		//Sleeping...
		//Sleeping...
		//Sleeping...
	   __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
		SystemClock_Config();
//	MX_RTC_Init();	//Test if this is really needed
		
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_4;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 49;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.InitValue = 0x0;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000202;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 10, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PMODE_GPIO_Port, PMODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VCC_DS_Pin|DS_DATA_Pin|CE_Pin|nCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VCC_SHT_GPIO_Port, VCC_SHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMODE_Pin */
  GPIO_InitStruct.Pin = PMODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PMODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VCC_DS_Pin DS_DATA_Pin CE_Pin nCS_Pin */
  GPIO_InitStruct.Pin = VCC_DS_Pin|DS_DATA_Pin|CE_Pin|nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VCC_SHT_Pin */
  GPIO_InitStruct.Pin = VCC_SHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VCC_SHT_GPIO_Port, &GPIO_InitStruct);

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
