/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ReceiveWiFiAT */
osThreadId_t ReceiveWiFiATHandle;
const osThreadAttr_t ReceiveWiFiAT_attributes = {
  .name = "ReceiveWiFiAT",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for SendWifiAT */
osThreadId_t SendWifiATHandle;
const osThreadAttr_t SendWifiAT_attributes = {
  .name = "SendWifiAT",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for RQ */
osMessageQueueId_t RQHandle;
const osMessageQueueAttr_t RQ_attributes = {
  .name = "RQ"
};
/* Definitions for SQ */
osMessageQueueId_t SQHandle;
const osMessageQueueAttr_t SQ_attributes = {
  .name = "SQ"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void ReceiveWifiATFunc(void *argument);
void SendWifiATFunc(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "queue.h"
int init_khalas = 0;
uint8_t s[16];

uint8_t termSign[1] = ";";
uint8_t orderSign[1] = "|";
char newLine[2] = "\n\r";

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of RQ */
  RQHandle = osMessageQueueNew (128, sizeof(uint8_t), &RQ_attributes);

  /* creation of SQ */
  SQHandle = osMessageQueueNew (128, sizeof(uint8_t), &SQ_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ReceiveWiFiAT */
  ReceiveWiFiATHandle = osThreadNew(ReceiveWifiATFunc, NULL, &ReceiveWiFiAT_attributes);

  /* creation of SendWifiAT */
  SendWifiATHandle = osThreadNew(SendWifiATFunc, NULL, &SendWifiAT_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	uint8_t innerBuffer[32];
	int seeker = 0;
  for(;;)
  {
		if(seeker > 31){
			seeker = 0;
		}
		
		if( xQueueReceive( RQHandle, &innerBuffer[seeker], portMAX_DELAY ) != pdPASS ){
			// nothing
		}else{
			taskENTER_CRITICAL();
			// HAL_UART_Transmit(&huart2, (uint8_t *)&innerBuffer[seeker], sizeof(uint8_t),1);
			if( xQueueSendToBack(SQHandle, &innerBuffer[seeker], 500) != pdPASS ) {}else {}
		
			if(innerBuffer[seeker] == termSign[0]){
				/*
				
				if( xQueueSendToBack(SQHandle, &newLine[0], 500) != pdPASS ){
				
				}else{
					
				}
				
				if( xQueueSendToBack(SQHandle, &newLine[1], 500) != pdPASS ) {
				
				} else {
					
				}
				*/
					
				seeker = 0;
			}else {
				seeker++;
			}
			taskEXIT_CRITICAL();
		}
		HAL_GPIO_TogglePin(GPIOB,PWR_PDCRB_PB3);
    osDelay(1);
  }

  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_ReceiveWifiATFunc */
/**
* @brief Function implementing the ReceiveWiFiAT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ReceiveWifiATFunc */
void ReceiveWifiATFunc(void *argument)
{
  /* USER CODE BEGIN ReceiveWifiATFunc */
	init_khalas = 1;
  /* Infinite loop */
  for(;;)
  {
		vTaskSuspend(NULL);
		taskENTER_CRITICAL();
		vTaskSuspend(SendWifiATHandle);
		
		/*
		HAL_UART_Transmit(&huart1, (uint8_t *)&newLine[0], sizeof(newLine),1);
		HAL_UART_Transmit(&huart1, s, sizeof(s),1);
		HAL_UART_Transmit(&huart1, (uint8_t *)&newLine[0], sizeof(newLine),1);
		*/
		/*
		
		if( xQueueSendToBack(RQHandle, s, 500) != pdPASS ) {
		}else {
			
		}
		*/
		//HAL_UART_Transmit(&huart1, (uint8_t *)&s[0], sizeof(uint8_t),1);
		//HAL_UART_Transmit(&huart1, (uint8_t *)&newLine[0], sizeof(newLine),1);
		
		for (int i= 0; i < 16 ; i++){
			if (s[i] != '\0'){
				taskENTER_CRITICAL();
				if( xQueueSendToBack(RQHandle, &s[i], 500) != pdPASS ) {
				}else {
					
				}
				s[i] = '\0';
				taskEXIT_CRITICAL();
			}
		}
		
		/*
		if( xQueueSendToBack(RQHandle, &orderSign[0], 500) != pdPASS ) {
				}else {
					
				}
		*/
		// HAL_UART_Transmit(&huart2, (uint8_t *)s, sizeof(s),1);
		
		vTaskResume(SendWifiATHandle);		
		taskEXIT_CRITICAL();
  }
  /* USER CODE END ReceiveWifiATFunc */
}

/* USER CODE BEGIN Header_SendWifiATFunc */
/**
* @brief Function implementing the SendWifiAT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendWifiATFunc */
void SendWifiATFunc(void *argument)
{
  /* USER CODE BEGIN SendWifiATFunc */
  /* Infinite loop */
	uint8_t innerBuffer[1];
  for(;;)
  {
    if( xQueueReceive( SQHandle, &innerBuffer[0], 1 ) != pdPASS ){
			// nothing
		}else{
			taskENTER_CRITICAL();
			HAL_UART_Transmit(&huart1, (uint8_t *)&innerBuffer[0], sizeof(uint8_t),1);
			taskEXIT_CRITICAL();
		}
		
		osDelay(1);
  }
  /* USER CODE END SendWifiATFunc */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
