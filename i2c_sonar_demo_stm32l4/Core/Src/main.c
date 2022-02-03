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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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

/* USER CODE BEGIN PV */
static const uint8_t SRF10_ADDR = 0xE0;

static const uint8_t FIRMWARE_REV_REG = 0x00;
static const uint8_t COMMAND_REG = 0x00;
static const uint8_t GAIN_REG = 0x01;
static const uint8_t RANGE_REG = 0x02;
static const uint8_t RANGE_HIGH_REG = 0x02;
static const uint8_t RANGE_LOW_REG = 0x03;

static const uint8_t RANGE_MODE_INCH = 0x50;			// range reading in [in]
static const uint8_t RANGE_MODE_CENTIMETER = 0x51;		// range reading in [cm]
static const uint8_t RANGE_MODE_MICROSECONDS = 0x52;	// range reading in [microsec]

static const uint32_t I2C_TIMEOUT = 5000;	// [milliseconds]

uint8_t i2c_data_buf[5];
HAL_StatusTypeDef ret;

uint8_t t_gain = 0x02;	// Max gain
double t_range = 2.0;	// Max range

uint8_t high_byte;
uint8_t low_byte;
uint16_t distance_value;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int fd, char* ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	return len;
}
void Read_SRF10_Firmware_Revision(uint8_t device_address);
void Initialize_SRF10(uint8_t device_address, uint8_t max_gain, double max_range);
HAL_StatusTypeDef SRF10_Set_Gain(uint8_t max_gain);
HAL_StatusTypeDef SRF10_Set_Range(double max_range);
HAL_StatusTypeDef SRF10_ReadRegister(uint8_t device_address, uint8_t register_address, uint8_t* data_buffer_ptr);
HAL_StatusTypeDef SRF10_WriteRegister(uint8_t device_address, uint8_t register_address, uint8_t value);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  if (HAL_I2C_IsDeviceReady(&hi2c1, SRF10_ADDR, 10, 5000) == HAL_OK)
  {
	  printf("Device is ready!\r\n");
	  Read_SRF10_Firmware_Revision(SRF10_ADDR);
	  Initialize_SRF10(SRF10_ADDR, t_gain, t_range);
  }
  else
  {
	  Error_Handler();
  }

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Setting up to get data in microseconds
	  if (SRF10_WriteRegister(SRF10_ADDR, COMMAND_REG, RANGE_MODE_CENTIMETER) != HAL_OK)
	  {
		  printf("Retrieving data ERROR\r\n");
		  Error_Handler();
	  }

	  // --- Reading sensor data ---
	  // Adding delay until register is finished reading
	  uint8_t command_register_value = 0xFF;
	  while (command_register_value == 0xFF)
	  {
		  SRF10_ReadRegister(SRF10_ADDR, COMMAND_REG, &command_register_value);
	  }

	  // Reading from high and low byte registers
	  if (SRF10_ReadRegister(SRF10_ADDR, RANGE_HIGH_REG, &high_byte) != HAL_OK)
	  {
		  printf("High byte ERROR\r\n");
		  Error_Handler();
	  }
	  if (SRF10_ReadRegister(SRF10_ADDR, RANGE_LOW_REG, &low_byte) != HAL_OK)
	  {
		  printf("Low byte ERROR\r\n");
		  Error_Handler();
	  }

	  distance_value = (((uint16_t)high_byte) << 8) + low_byte;
	  printf("Range: %hu cm\r\n", distance_value);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// Reads firmware register
void Read_SRF10_Firmware_Revision(uint8_t device_address)
{
	uint8_t buf[5];

	// Telling to read from firmware revision register
	SRF10_ReadRegister(device_address, FIRMWARE_REV_REG, buf);

	printf("%X sonar firmware revision: %d\r\n", device_address, buf[0]);
}

/*
 * Initializes sonar sensor at device address by initializing the
 * gains and maximum range.
 *
 */
void Initialize_SRF10(uint8_t device_address, uint8_t max_gain, double max_range)
{
	if (SRF10_Set_Gain(max_gain) != HAL_OK)
	{
		printf("Setting gain ERROR\r\n");
		Error_Handler();
	}
	if (SRF10_Set_Range(max_range) != HAL_OK)
	{
		printf("Setting range ERROR\r\n");
		Error_Handler();
	}
	printf("%X sonar initialized\r\n", device_address);
}

/*
 * Sets the gain of the SRF10 sonar sensor.
 *
 */
HAL_StatusTypeDef SRF10_Set_Gain(uint8_t max_gain)
{
	if (SRF10_WriteRegister(SRF10_ADDR, GAIN_REG, max_gain) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/*
 * Sets the range of the SRF10 sonar sensor.
 *
 */
HAL_StatusTypeDef SRF10_Set_Range(double max_range)
{
	// Range limits
	if (max_range > 6.0 || max_range < 0.0)
	{
		max_range = 6.0;
	}

	uint8_t range_command = (uint8_t)((max_range - 0.043)*23.26);
	if (SRF10_WriteRegister(SRF10_ADDR, RANGE_REG, range_command) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

/*
 * Reads from the register of SRF10 using I2C
 *
 */
HAL_StatusTypeDef SRF10_ReadRegister(uint8_t device_address, uint8_t register_address, uint8_t* data_buffer_ptr)
{
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)device_address, &register_address, 1, I2C_TIMEOUT) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)device_address, data_buffer_ptr, 1, I2C_TIMEOUT) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

/*
 * Writes to the register of SRF10 using I2C
 *
 */
HAL_StatusTypeDef SRF10_WriteRegister(uint8_t device_address, uint8_t register_address, uint8_t value)
{
	uint8_t command_buffer[2];
	command_buffer[0] = register_address;
	command_buffer[1] = value;

	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)device_address, command_buffer, 2, I2C_TIMEOUT) != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
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
//	  printf("ERROR!!!\r\n");
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
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

