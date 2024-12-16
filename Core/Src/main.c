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
#include "dfsdm.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TFT_CS_LOW()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define TFT_CS_HIGH()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define TFT_DC_LOW()     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define TFT_DC_HIGH()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)
#define TFT_RST_LOW()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET)
#define TFT_RST_HIGH()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t dfdsdm_buffer[AUDIO_BUFFER_SIZE];  // DFSDM raw data buffer
float volume_buffer[AUDIO_BUFFER_SIZE];   // Volume in decibels
volatile uint8_t data_ready = 0;          // Flag for new volume data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void TFT_Init(void);
void TFT_Write(uint8_t data);
void TFT_SendCommand(uint8_t cmd);
void TFT_SendData(uint8_t data);
void TFT_Clear(uint16_t color);
void TFT_DrawGraph(float *data, uint16_t size, uint16_t color);
void Process_Audio_Data(void);
float Calculate_dB(int32_t *buffer, uint16_t size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Send data to the display via SPI */
void TFT_Write(uint8_t data)
{
  HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
}

/* Send a command to the display */
void TFT_SendCommand(uint8_t cmd)
{
  TFT_DC_LOW();
  TFT_CS_LOW();
  TFT_Write(cmd);
  TFT_CS_HIGH();
}

/* Send data to the display */
void TFT_SendData(uint8_t data)
{
  TFT_DC_HIGH();
  TFT_CS_LOW();
  TFT_Write(data);
  TFT_CS_HIGH();
}

/* Initialize the TFT Display */
void TFT_Init(void)
{
  // Reset the display
  TFT_RST_LOW();
  HAL_Delay(10);
  TFT_RST_HIGH();
  HAL_Delay(120);

  // Initialization commands for ILI9341
  TFT_SendCommand(0xEF);
  TFT_SendData(0x03);
  TFT_SendData(0x80);
  TFT_SendData(0x02);

  TFT_SendCommand(0xCF);
  TFT_SendData(0x00);
  TFT_SendData(0xC1);
  TFT_SendData(0x30);
}

void TFT_Clear(uint16_t color)
{
  uint16_t x, y;
  for (x = 0; x < 240; x++) {
    for (y = 0; y < 320; y++) {
      TFT_SendCommand(0x2A); // Set Column Address
      TFT_SendData(x >> 8);
      TFT_SendData(x & 0xFF);
      TFT_SendCommand(0x2B); // Set Row Address
      TFT_SendData(y >> 8);
      TFT_SendData(y & 0xFF);
      TFT_SendCommand(0x2C); // Write Memory Start
      TFT_SendData(color >> 8);
      TFT_SendData(color & 0xFF);
    }
  }
}

void TFT_DrawGraph(float *data, uint16_t size, uint16_t color)
{
  uint16_t x, y;
  for (x = 0; x < size; x++) {
    y = 320 - (uint16_t) (data[x] * 10);  // Scale dB data to screen height
    TFT_SendCommand(0x2A); // Set Column Address
    TFT_SendData(x >> 8);
    TFT_SendData(x & 0xFF);
    TFT_SendCommand(0x2B); // Set Row Address
    TFT_SendData(y >> 8);
    TFT_SendData(y & 0xFF);
    TFT_SendCommand(0x2C); // Write Memory Start
    TFT_SendData(color >> 8);
    TFT_SendData(color & 0xFF);
  }
}

/* Process Audio Data: Converts raw samples into decibels */
void Process_Audio_Data(void)
{
  float db_value = Calculate_dB(dfdsdm_buffer, AUDIO_BUFFER_SIZE);

  // Store decibel value into the volume buffer
  static uint16_t index = 0;
  volume_buffer[index++] = db_value;
  if (index >= AUDIO_BUFFER_SIZE) {
    index = 0;  // Wrap around
  }
}

/* Calculate the average volume in decibels */
float Calculate_dB(int32_t *buffer, uint16_t size)
{
  float sum_square = 0.0f;

  for (uint16_t i = 0; i < size; i++) {
    sum_square += (float) buffer[i] * (float) buffer[i];
  }

  float rms = sqrtf(sum_square / size);
  float db = 20.0f * log10f(rms + 1e-5); // Add small value to avoid log(0)
  return db;
}

/* Timer Callback: Set data_ready flag every 500ms */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2) {
    data_ready = 1;
  }
}

/* DMA Error Callback */
void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  Error_Handler();
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, (int32_t*) dfdsdm_buffer,
  AUDIO_BUFFER_SIZE);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    if (data_ready) {
      data_ready = 0;

      // Process audio data
      Process_Audio_Data();

      // Clear the display
      TFT_Clear(0x0000);  // Black background

      // Draw the graph
      TFT_DrawGraph(volume_buffer, AUDIO_BUFFER_SIZE, 0xFFFF);  // White graph
    }
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
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
      | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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
  while (1) {
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
