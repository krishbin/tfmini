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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdbool.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t tfmini_left_rawserial[9];
static uint8_t tfmini_right_rawserial[9];
static bool tfmini_left_hasdata = false;
static bool tfmini_right_hasdata = false;
static int tfmini_left_receivedtime = 0;
static int tfmini_right_receivedtime = 0;
static int tfmini_left_data[2];
static int tfmini_right_data[2];

bool decode_tfmini_packet(uint8_t *raw,int *decoded_data){
  decoded_data[0] = 0;
  decoded_data[1] = 0;
  int checksum = 0;
  if( raw[0] == 0x59 && raw[1] == 0x59 ){
    for(int i = 0; i < 8 ; ++i ){
      checksum += raw[i];
    }
    if( raw[8] == (checksum % 256) ){
      decoded_data[0] = raw[2] + raw[3] * 256;
      decoded_data[1] = raw[4] + raw[5] * 256;
      return true;
    }
  }
  return false;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_0);  // toggle PA0
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
  if(huart->Instance == UART4){
    if(decode_tfmini_packet(tfmini_left_rawserial,tfmini_left_data)){
      tfmini_left_hasdata = true;
      tfmini_left_receivedtime = HAL_GetTick();
    }
    HAL_UART_Receive_DMA(&huart4, tfmini_left_rawserial, 9); 
  }
  if(huart->Instance == USART3){
    if(decode_tfmini_packet(tfmini_right_rawserial,tfmini_right_data)){
      tfmini_right_hasdata = true;
      tfmini_right_receivedtime = HAL_GetTick();
    }
    HAL_UART_Receive_DMA(&huart3, tfmini_right_rawserial, 9); 
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
  MX_DMA_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_DMA(&huart4, tfmini_left_rawserial, 9); 
  HAL_UART_Receive_DMA(&huart3, tfmini_right_rawserial, 9); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    int current = HAL_GetTick();
    if( (current - tfmini_left_receivedtime) > 100 ){
      tfmini_left_hasdata = false;
    }
    if( (current - tfmini_right_receivedtime) > 100 ){
      tfmini_right_hasdata = false;
    }
    if(tfmini_left_hasdata){
      printf("left d: %d s: %d ",tfmini_left_data[0],tfmini_left_data[1]);
    }
    if(tfmini_left_hasdata){
      printf("right d: %d s: %d ",tfmini_right_data[0],tfmini_right_data[1]);
    }

    if(tfmini_left_hasdata || tfmini_right_hasdata){
      printf("\n");
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
