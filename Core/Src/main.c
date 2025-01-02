/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LoRa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern SPI_HandleTypeDef hspi1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
LoRa myLoRa;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t readLevel(void)
{
	uint8_t ret = 0;
	HAL_GPIO_WritePin(COMMON_CONTROL_GPIO_Port, COMMON_CONTROL_Pin, 0);
	HAL_Delay(300);
	if(!HAL_GPIO_ReadPin(LEVEL1_GPIO_Port, LEVEL1_Pin))
		ret |= 0x01;
	if(!HAL_GPIO_ReadPin(LEVEL2_GPIO_Port, LEVEL2_Pin))
		ret |= 0x02;
	if(!HAL_GPIO_ReadPin(LEVEL3_GPIO_Port, LEVEL3_Pin))
		ret |= 0x04;
	if(!HAL_GPIO_ReadPin(LEVEL4_GPIO_Port, LEVEL4_Pin))
		ret |= 0x08;
	HAL_GPIO_WritePin(COMMON_CONTROL_GPIO_Port, COMMON_CONTROL_Pin, 1);
	HAL_Delay(300);
	return ret;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t levelVar, bufTx[5];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  myLoRa = newLoRa();
  myLoRa.CS_port         = RF_CS_GPIO_Port;
  myLoRa.CS_pin          = RF_CS_Pin;
  myLoRa.reset_port      = RF_RST_GPIO_Port;
  myLoRa.reset_pin       = RF_RST_Pin;
  myLoRa.DIO0_port       = RF_D0_GPIO_Port;
  myLoRa.DIO0_pin        = RF_D0_Pin;
  myLoRa.hSPIx           = &hspi1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  LoRa_init(&myLoRa);
  HAL_Delay(500);
  LoRa_startReceiving(&myLoRa);
  /*uint8_t received_data[20];
  uint8_t packet_size = 0;*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart1, (const uint8_t *)"Level Tx unit", 13, 100);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for(int i = 0; i < 3; i++)
	  {
		  //packet_size = LoRa_receive(&myLoRa, received_data, 20);
		  /*if(packet_size > 0)
		  {
			  HAL_UART_Transmit(&huart1, (const uint8_t *)received_data, packet_size, 100);
		  }*/
		  HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, 0);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(HEART_BEAT_GPIO_Port, HEART_BEAT_Pin, 1);
		  HAL_Delay(1000);
	  }
	  levelVar = readLevel();
	  bufTx[0] = (levelVar/10)|0x30;
	  bufTx[1] = (levelVar%10)|0x30;
	  LoRa_transmit(&myLoRa, (uint8_t*)bufTx, 2, 100);
	 // HAL_UART_Transmit(&huart1, (const uint8_t *)&levelVar, 1, 100);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV8;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
