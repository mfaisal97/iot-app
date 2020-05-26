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
I2C_HandleTypeDef hi2c1;

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
/* Definitions for HandleRTCTime */
osThreadId_t HandleRTCTimeHandle;
const osThreadAttr_t HandleRTCTime_attributes = {
  .name = "HandleRTCTime",
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
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void ReceiveWifiATFunc(void *argument);
void SendWifiATFunc(void *argument);
void HandleRTCTimeFunc(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "queue.h"
int init_khalas = 0;
uint8_t s[16];

uint8_t termSign[1] = ";";
uint8_t commandStart[2] = "AT";
uint8_t ledArgument[3] = "LED";
uint8_t timeArgument[4] = "TIME";

uint8_t orderSign[1] = "|";
uint8_t querySign[1] = "?";
uint8_t assignSign[1] = "=";
char newLine[2] = "\n\r";

uint8_t ledIsOn[2] = "On";
uint8_t ledIsOff[3] = "Off";

int timeParts[3] = {0,0,0};

int RTCOp = 0;

int getNextDigit(uint8_t* str, int sz, int* res){
	int sign = 1;
	for (int i = 0; i <= sz; i++){
		if(str[i] == '-'){
			sign = sign * -1;
		}else if(str[i]>= '0' & str[i] <= '9'){
			*res = sign*(str[i] - '0');
			return i;
		}else if(str[i] != ' ' && str[i] != '\t' && str[i] != '+'){
			return -1;
		}
	}
	return -1;
}

int matchNextStr(uint8_t* str, int sz, uint8_t* str2, int sz2){
	for (int i = 0; i < sz2; i++){
		if (i > sz || str[i]!= str2[i]){	
			return -1;
		}
	}
	return sz2;
}

int getNextOpSign(uint8_t* str, int sz, int* res){
	for (int i = 0; i < sz; i++){
		if(str[i] == '-'){
			*res  = 0;
			return i;
		}else if(str[i]>= '+'){
			*res = 1;
			return i;
		}else if(str[i] != ' ' && str[i] != '\t'){
			return -1;
		}
	}
	return -1;
}	


uint8_t hexToAscii(uint8_t n)//4-bit hex value converted to an ascii character
{
 if (n>=0 && n<=9) n = n + '0';
 else n = n - 10 + 'A';
 return n;
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
  MX_I2C1_Init();
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

  /* creation of HandleRTCTime */
  HandleRTCTimeHandle = osThreadNew(HandleRTCTimeFunc, NULL, &HandleRTCTime_attributes);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
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
  hi2c1.Init.Timing = 0x00000E14;
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
			//if( xQueueSendToBack(SQHandle, &innerBuffer[seeker], 500) != pdPASS ) {}else {}
		
			int baseInd = 0;
			if(innerBuffer[seeker] == termSign[0]){
				int atInd, ledInd, timeInd;
				
				atInd = matchNextStr(innerBuffer, seeker - baseInd, commandStart, 2);
				if ( atInd != -1){
					baseInd = atInd;
					ledInd = matchNextStr(&innerBuffer[baseInd], seeker - baseInd, ledArgument, 3);
					timeInd = matchNextStr(&innerBuffer[baseInd], seeker - baseInd, timeArgument, 4);
					
					int queryInd, assignInd;
					if(ledInd != -1 ){
						baseInd = baseInd+ledInd;
						queryInd = matchNextStr(&innerBuffer[baseInd], seeker - baseInd, querySign, 1);
						assignInd = matchNextStr(&innerBuffer[baseInd], seeker - baseInd, assignSign, 1);
						
						if(queryInd!=-1){
							int res = HAL_GPIO_ReadPin(GPIOB,PWR_PDCRB_PB3);
							if (res){
								HAL_UART_Transmit(&huart1, ledIsOn, sizeof(ledIsOn),1);
							}else {
								HAL_UART_Transmit(&huart1, ledIsOff, sizeof(ledIsOff),1);
							}
						}else if (assignInd != -1){
							baseInd = baseInd + assignInd;
							int newVal = 0;
							int newValInd = getNextDigit(&innerBuffer[baseInd], seeker-baseInd, &newVal);
							if (newValInd!=-1){
								HAL_GPIO_WritePin(GPIOB, PWR_PDCRB_PB3, newVal);
							}
						}
						
					}else if (timeInd != -1){
						baseInd = baseInd+timeInd;
						queryInd = matchNextStr(&innerBuffer[baseInd], seeker - baseInd, querySign, 1);
						assignInd = matchNextStr(&innerBuffer[baseInd], seeker - baseInd, assignSign, 1);
						
						if(queryInd!=-1){
							RTCOp = 3;
							vTaskResume(HandleRTCTimeHandle);
						}else if (assignInd != -1){
							baseInd = baseInd + assignInd;
							int RTCOpInd = getNextDigit(&innerBuffer[baseInd], seeker-baseInd, &RTCOp);
							
							if (RTCOpInd != -1){
								baseInd = baseInd + 1;
								int a,b;
								int aInd = 0, bInd = 0;
								int done = 1;
								
								for (int i =0; i < 3; i++){
									aInd = getNextDigit(&innerBuffer[baseInd], seeker-baseInd, &a);
									bInd = getNextDigit(&innerBuffer[baseInd + 1], seeker - baseInd - 1, &b);
									
									if (aInd != -1 && bInd !=-1){
										timeParts[i] = a*10 + b;
										baseInd = baseInd + 1 + 1+1;
									}else {
										done = 0;
										break;
									}
								}
								
								if (done){
									vTaskResume(HandleRTCTimeHandle);
								}
							}
						}
					}else {
						
					}
				}
				
				
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
		
		if (seeker == 0){
			osDelay(30);
		}
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

/* USER CODE BEGIN Header_HandleRTCTimeFunc */
/**
* @brief Function implementing the HandleRTCTime thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HandleRTCTimeFunc */
void HandleRTCTimeFunc(void *argument)
{
  /* USER CODE BEGIN HandleRTCTimeFunc */
  //Transmit via I2C
	uint8_t secbuffer [2], minbuffer [2], hourbuffer [2];
	// seconds
	secbuffer[0] = 0x00; //register address
	secbuffer[1] = 0x01; //data to put in register --> 0 sec
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, secbuffer, 2, 10);
	// minutes
	minbuffer[0] = 0x01; //register address
	minbuffer[1] = 0x00; //data to put in register --> 15 min
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, minbuffer, 2, 10);
	// hours
	hourbuffer[0] = 0x02; //register address
	hourbuffer[1] = 0x00; //data to put in register 0100 1001 --> 7 am
	HAL_I2C_Master_Transmit(&hi2c1, 0xD0, hourbuffer, 2, 10);
	//Receive via I2C and forward to UART
	uint8_t out[] = {0,0,':',0,0,':',0,0,'\r','\n'};
	
	
	uint8_t alarmsecbuffer [2], alarmminbuffer [2], alarmhourbuffer [2], alaramdate[2], alarmcontrol[2], alarmstatus[2];
	//register address
	alarmsecbuffer[0] = 0x07;
	alarmminbuffer[0] = 0x08;
	alarmhourbuffer[0] = 0x09;
	
	//date
	alaramdate[0] = 0x0A; 
	alaramdate[1] = 0x80; 
	//control
	alarmcontrol[0] = 0x0e;
	alarmcontrol[1] = 0x1d;
	//status
	alarmstatus[0] = 0x0f;
	alarmstatus[1] = 0x88;
		
  /* Infinite loop */
  for(;;)
  {
		vTaskSuspend(NULL);
		
		if (RTCOp == 1 || RTCOp == 2){
			secbuffer[1] = timeParts[2] % 10;
			secbuffer[1] = secbuffer[1] + timeParts[2] / 10 * 16;
			
			minbuffer[1] = timeParts[1] % 10;
			minbuffer[1] = minbuffer[1] + timeParts[1] / 10 * 16;
			
			/*
			if(timeParts[0] > 12){
				hourbuffer[1] = 0x60;
				timeParts[0] = timeParts[0] - 12;
			}else {
				hourbuffer[1] = 0x40;
			}
			*/
			
			
			hourbuffer[1] = 0 + timeParts[0] % 10;
			hourbuffer[1] = hourbuffer[1] + timeParts[0] / 10 * 16;
			
		}
		
		if (RTCOp == 1){
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, secbuffer, 2, 10);
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, minbuffer, 2, 10);
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, hourbuffer, 2, 10);
		}else if (RTCOp == 2){
			
			alarmsecbuffer[1] = secbuffer[1];
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarmsecbuffer, 2, 10);

			alarmminbuffer[1] = minbuffer[1];
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarmminbuffer, 2, 10);
			
			alarmhourbuffer[1] = hourbuffer[1];
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarmhourbuffer, 2, 10);
			
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alaramdate, 2, 10);
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarmcontrol, 2, 10);
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, alarmstatus, 2, 10);			
			
		}else if (RTCOp == 3){
			//send seconds register address 00h to read from
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, secbuffer, 1, 10);
			//read data of register 00h to secbuffer[1]
			HAL_I2C_Master_Receive(&hi2c1, 0xD1, secbuffer+1, 1, 10);
			//prepare UART output
			out[6] = hexToAscii(secbuffer[1] >> 4 );
			out[7] = hexToAscii(secbuffer[1] & 0x0F );
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, minbuffer, 1, 10);
			HAL_I2C_Master_Receive(&hi2c1, 0xD1, minbuffer+1, 1, 10);
			out[3] = hexToAscii(minbuffer[1] >> 4 );
			out[4] = hexToAscii(minbuffer[1] & 0x0F );
			HAL_I2C_Master_Transmit(&hi2c1, 0xD0, hourbuffer, 1, 10);
			HAL_I2C_Master_Receive(&hi2c1, 0xD1, hourbuffer+1, 1, 10);
			out[0] = hexToAscii((hourbuffer[1] >> 4)&0x7);
			out[1] = hexToAscii(hourbuffer[1] & 0x0F);
			
			// transmit time to UART
			HAL_UART_Transmit(&huart1,out, sizeof(out), 10);			
		}
  }
  /* USER CODE END HandleRTCTimeFunc */
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
