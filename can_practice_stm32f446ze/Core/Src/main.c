/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_task.h"
#include <string.h>
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
uint8_t CAN1_TxData[8];
uint8_t CAN2_TxData[8];
uint8_t RxData0[8];
uint8_t RxData1[8];
CAN_RxHeaderTypeDef CAN1_RxHeader;

int rxfifo = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Blink_LED(const char* led_color, uint16_t on_duration, uint16_t off_duration);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void LedSteup()
{
	// Activate RX FIFO for each CAN
	HAL_CAN_Start(&hcan1);
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_CAN_Start(&hcan2);
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}
}

/*
 * Sends a message over the CAN bus from CAN2 to CAN1
 */
void LedLoop()
{
	uint8_t RxData[8];

	// Transmitting message over CAN2
	CAN2_TxData[0] = 50;
	CAN2_TxData[1] = 0xAA;
	CAN2_TxData[2] = 0x32;
	CAN2_TxData[3] = 0x55;

	CAN_write(&hcan2, 0x103, 4, CAN2_TxData);
	HAL_Delay(1);	// Delay so that message can be received by nodes

	if (rxfifo == 0)
	{
		memcpy(RxData, RxData0, sizeof(RxData0));
	}
	else if (rxfifo == 1)
	{
		memcpy(RxData, RxData1, sizeof(RxData1));
	}

	// Reading the message transmitted from CAN2
	if (CAN1_RxHeader.StdId == 0x103 && RxData[0] == 50)
	{
		// Blink the GREEN LED
		Blink_LED("green", 500, 500);
	}

	HAL_Delay(1);

	// Transmitting message over CAN1
	CAN1_TxData[0] = 20;
	CAN1_TxData[1] = 0xAA;

	CAN_write(&hcan1, 0x446, 2, CAN1_TxData);
	HAL_Delay(1);	// Delay so that message can be received by nodes

	if (rxfifo == 0)
	{
		memcpy(RxData, RxData0, sizeof(RxData0));
	}
	else if (rxfifo == 1)
	{
		memcpy(RxData, RxData1, sizeof(RxData1));
	}

	// Reading the message transmitted from CAN1
	if (CAN1_RxHeader.StdId == 0x446 && RxData[0] == 20)
	{
		// Blink the BLUE LED
		Blink_LED("blue", 500, 500);
	}

	HAL_Delay(1);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  LedSteup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  LedLoop();
	  HAL_Delay(1000);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*
 * @brief Blinks the onboard LEDs for a specified duration
 *
 * @param led_color color of onboard LED to blink (blue, green, or red)
 * @param on_duration duration of LED staying on [milliseconds]
 * @param off_duratin duration of LED staying off [milliseconds]
 *
 * @retval None
 */
void Blink_LED(const char* led_color, uint16_t on_duration, uint16_t off_duration)
{
	if (!strcmp(led_color, "blue"))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		HAL_Delay(on_duration);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		HAL_Delay(off_duration);
	}
	else if (!strcmp(led_color, "green"))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
		HAL_Delay(on_duration);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
		HAL_Delay(off_duration);
	}
	else if (!strcmp(led_color, "red"))
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
		HAL_Delay(on_duration);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
		HAL_Delay(off_duration);
	}
	else
	{
		Error_Handler();
	}
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
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);	// Light up the red LED
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

