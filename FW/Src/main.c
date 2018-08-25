
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "onewire.h"
#include "lcd-nokia1100.h"
#include "PID.h"
#include "driverButton.h"
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osThreadId startpwmdemo;
osThreadId interfacetask;
osThreadId displaytask;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void const * argument);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t pulse_width = 0;
uint32_t Triac_angle = 3000;
float Temperature = 0.0f;
float Power = 0;
float Setpoint = 20.0;
uint8_t Standby = 1;

// PID controllers
struct pid_controller pidctrl;
pid_t pid = 0;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void StartPwmDemo(void const * argument);
void InterfaceTask (void const * argument);
void DisplayTask(void const * argument);

#define LOWER_LIMIT 0
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if(GPIO_Pin== GPIO_PIN_9) 
  {
    
  __HAL_TIM_SET_COUNTER(&htim1, Triac_angle);
 
  if (Triac_angle == 900)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  else 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 50);
    
    
  } 

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  
  OW_Init();
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(startpwmDemo, StartPwmDemo, osPriorityNormal, 0, 128);
  startpwmdemo = osThreadCreate(osThread(startpwmDemo), NULL);
  
  initButton();
     
  osThreadDef(interfaceTask, InterfaceTask, osPriorityNormal, 0, 128);
  interfacetask = osThreadCreate(osThread(interfaceTask), NULL);
  
  osThreadDef(displayTask, DisplayTask, osPriorityNormal, 0, 128);
  displaytask = osThreadCreate(osThread(displayTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 11000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 256;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_CLK_Pin|LCD_SDA_Pin|LCD_CS_Pin|LCD_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_CLK_Pin LCD_SDA_Pin LCD_CS_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_CLK_Pin|LCD_SDA_Pin|LCD_CS_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ZC_IN_Pin */
  GPIO_InitStruct.Pin = ZC_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ZC_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT_PLUS_Pin */
  GPIO_InitStruct.Pin = BUT_PLUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUT_PLUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUT_MINUS_Pin */
  GPIO_InitStruct.Pin = BUT_MINUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUT_MINUS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
const unsigned char start_logo [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F,
0xE7, 0x1C, 0x70, 0xC0, 0x00, 0x7E, 0xEE, 0x3C, 0xF0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x78, 0xD0, 0xB0, 0x60, 0x40, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0xC0, 0x8F, 0x7F, 0xCC, 0x18,
0x33, 0x6E, 0xF8, 0xE1, 0xC3, 0x86, 0x0F, 0x3C, 0x20, 0x63, 0xC7, 0x8C, 0x18, 0x30, 0x60, 0xF8,
0xF0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x0E, 0x1C, 0x3C, 0x6C,
0x48, 0xD8, 0x9B, 0x92, 0x14, 0x3C, 0x38, 0x31, 0x21, 0x62, 0xC2, 0x86, 0xB7, 0x7F, 0xDF, 0x9B,
0x16, 0x36, 0x24, 0x2D, 0x69, 0x4F, 0x4F, 0xCE, 0xDC, 0x18, 0x18, 0x11, 0x33, 0x36, 0x20, 0xA0,
0xE3, 0x6E, 0x59, 0xF3, 0xE6, 0x84, 0x08, 0x18, 0x30, 0x60, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
0x80, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x07,
0x07, 0x0F, 0x0B, 0x3B, 0x73, 0xF3, 0xB2, 0xA6, 0x26, 0x6C, 0x6C, 0x6C, 0x6D, 0x6F, 0x7B, 0xF3,
0x73, 0x73, 0x63, 0x22, 0x20, 0x20, 0x26, 0x26, 0x27, 0xEF, 0xFF, 0x3F, 0x3F, 0x63, 0x41, 0xF7,
0xFC, 0x0C, 0x18, 0x10, 0xF0, 0xB1, 0x03, 0x06, 0x0C, 0x30, 0xE0, 0x00, 0x01, 0x06, 0x1C, 0xF0,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x7C, 0x46, 0x03, 0x01, 0x01,
0x00, 0x00, 0x01, 0x01, 0x03, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x0B, 0x0B, 0x1B, 0x13, 0x71, 0xD9, 0xD9, 0x89,
0xDF, 0xF6, 0x76, 0x36, 0xF6, 0xB2, 0x92, 0x92, 0x9B, 0x9F, 0xDF, 0x7E, 0x7C, 0x3C, 0xBF, 0xFF,
0x78, 0x71, 0x71, 0xF3, 0x3F, 0x07, 0x06, 0x00, 0xC0, 0x70, 0x9F, 0xC0, 0xB0, 0x98, 0x87, 0xC1,
0x40, 0x60, 0x20, 0x30, 0x18, 0x08, 0x0C, 0x06, 0x03, 0x81, 0xC0, 0x60, 0x30, 0xCC, 0xC6, 0x60,
0x30, 0x1C, 0x06, 0x06, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x0F, 0xF9, 0xB0,
0x18, 0x88, 0x8C, 0x84, 0xC6, 0x63, 0x31, 0x1B, 0x8E, 0xFE, 0x3E, 0x3A, 0xBB, 0x79, 0x19, 0xEC,
0xE6, 0x63, 0x10, 0xFB, 0x04, 0x08, 0xF0, 0x02, 0x19, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
0x00, 0x40, 0x20, 0xB0, 0xD0, 0x48, 0x64, 0x36, 0x1B, 0x0D, 0x04, 0x06, 0x03, 0x01, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
0x01, 0x01, 0x03, 0x06, 0x06, 0x02, 0x0E, 0x0B, 0x09, 0x0C, 0x1E, 0xD3, 0xF9, 0x1C, 0x16, 0x13,
0x18, 0x0C, 0x06, 0x03, 0x02, 0x0B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xFE, 0xF2,
0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x1C, 0x18, 0x30,
0x20, 0x30, 0x20, 0x20, 0x60, 0x30, 0x38, 0x38, 0x38, 0x34, 0x1C, 0x1B, 0x0E, 0x03, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char troll [] = 
{
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0x60, 0x60,
0xE0, 0xE0, 0x60, 0xF0, 0xF0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0xB0, 0x30, 0x70, 0x70, 0x70, 0x70,
0x70, 0x70, 0x70, 0x70, 0x50, 0x50, 0x50, 0x50, 0x10, 0xB0, 0xB0, 0x30, 0x30, 0x30, 0x30, 0x30,
0x30, 0x30, 0x30, 0x30, 0x60, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xF0, 0xF8, 0x1C, 0x0E, 0x06, 0x03, 0x03, 0x01, 0x41, 0x22, 0x12, 0x10, 0x02, 0x0A, 0x0A,
0x0A, 0x0A, 0x0A, 0x0A, 0x52, 0x12, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x01, 0x61, 0x11,
0x01, 0x01, 0x05, 0x05, 0x05, 0x85, 0x81, 0x8A, 0x8A, 0x8A, 0x04, 0x15, 0x19, 0x2A, 0x52, 0x14,
0x28, 0x48, 0x10, 0x00, 0x00, 0x00, 0x01, 0x03, 0x0F, 0x3C, 0xF8, 0xC0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE0, 0xF0, 0xB8, 0x5C,
0xAF, 0x53, 0x51, 0x80, 0x80, 0x80, 0x88, 0x88, 0x80, 0x00, 0x0C, 0x1C, 0x1E, 0x1E, 0x1E, 0x1E,
0x1E, 0x1C, 0x3C, 0x3C, 0xF8, 0xF0, 0x30, 0x30, 0x10, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3C, 0x7E,
0x66, 0x33, 0x33, 0x3B, 0x19, 0x1F, 0x1F, 0x3F, 0xFF, 0xCF, 0x9F, 0x9F, 0x9F, 0x1E, 0x1C, 0x98,
0x80, 0x88, 0xD8, 0xC8, 0xE0, 0x60, 0x60, 0x60, 0x68, 0x68, 0xC8, 0xCB, 0x9F, 0x1E, 0x3C, 0x18,
0x30, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1F, 0x7F, 0xFF, 0xE0,
0x4F, 0x50, 0x81, 0x01, 0xC1, 0xF1, 0xFD, 0xF9, 0x83, 0x07, 0x07, 0x06, 0x47, 0x43, 0x20, 0x30,
0x70, 0x78, 0x7C, 0xCE, 0x87, 0x07, 0x00, 0x80, 0x00, 0x00, 0x40, 0x60, 0x60, 0x60, 0x68, 0x4C,
0x8C, 0xF8, 0xF8, 0x08, 0x10, 0x10, 0x90, 0x90, 0x90, 0xC0, 0xC1, 0xC1, 0xE1, 0xE1, 0xE1, 0xF1,
0x31, 0x39, 0x19, 0x1C, 0x8C, 0xCE, 0xFF, 0xFC, 0x18, 0x18, 0x30, 0x41, 0x7F, 0x3F, 0x00, 0xC0,
0x00, 0x80, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3F,
0xFE, 0x00, 0x00, 0x18, 0xFF, 0xFF, 0xFF, 0xE3, 0xFF, 0xFF, 0xC7, 0xC6, 0xFE, 0xFC, 0xCC, 0x8C,
0x8C, 0x8C, 0x98, 0xF9, 0xF9, 0x9B, 0x9B, 0x99, 0x8C, 0x8C, 0xFC, 0xFC, 0x8C, 0x8C, 0x8C, 0x86,
0xC6, 0xC6, 0xC6, 0xFF, 0xFF, 0xE3, 0xF1, 0xF1, 0x71, 0x71, 0x38, 0x38, 0x3C, 0x7E, 0xFF, 0x9F,
0xC6, 0xE6, 0x77, 0x3F, 0x0F, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0x38, 0x1D, 0x0E,
0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
0xFF, 0x00, 0x00, 0x00, 0x07, 0x3F, 0x7F, 0xFF, 0xFF, 0xD7, 0xDF, 0xFF, 0xEF, 0x8F, 0x9F, 0xFF,
0xEF, 0x8F, 0x0F, 0x1F, 0x7F, 0xFF, 0x0F, 0x0F, 0x0F, 0x07, 0x07, 0xFF, 0xFF, 0x87, 0x83, 0x83,
0x83, 0x81, 0x81, 0xC1, 0xC3, 0xCF, 0xFC, 0x70, 0x70, 0x30, 0x38, 0x9C, 0x8C, 0x4E, 0x47, 0x83,
0x81, 0xC0, 0x40, 0x00, 0x80, 0xC0, 0xE0, 0x60, 0x30, 0x1C, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF,
0x83, 0x00, 0x00, 0x0C, 0x20, 0x20, 0x40, 0x44, 0x88, 0x99, 0x91, 0x91, 0x81, 0xA1, 0xA1, 0x83,
0x03, 0x03, 0x0B, 0x0B, 0x0B, 0x0B, 0x0B, 0x03, 0x03, 0x03, 0x43, 0x43, 0x43, 0x43, 0x13, 0x31,
0x31, 0x21, 0x09, 0x19, 0x94, 0x94, 0xCE, 0xCA, 0x6A, 0x65, 0x35, 0x32, 0x1A, 0x1B, 0x0D, 0x0D,
0x06, 0x06, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
0x07, 0x0E, 0x1C, 0x18, 0x38, 0x30, 0x30, 0x30, 0x70, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60,
0x30, 0x30, 0x30, 0x30, 0x30, 0x10, 0x18, 0x18, 0x18, 0x1C, 0x0C, 0x0C, 0x0C, 0x0E, 0x0E, 0x06,
0x06, 0x06, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0xFE, 
};

uint32_t status=0;
float temp=0;
float temp2=0;
uint32_t limit=0;
uint32_t work_time=25;	
uint8_t buf[2];
uint16_t* t=(uint16_t*)buf;
char str[16];

int s,m,e;

//void getSME( unsigned int  s, unsigned int m, unsigned int  e, float number )
//{
//    unsigned int* ptr = (unsigned int*)&number;
//
//    s = *ptr >> 31;
//    e = *ptr & 0x7f800000;
//    e >>= 23;
//    m = *ptr & 0x007fffff;
//}
#define N_DECIMAL_POINTS_PRECISION (1000) // n = 3. Three decimal points.

//void update_temp(void)
//{
//  //float spd  = 88.8;
//  
//	if ((int)temp / 10 != 0)
//		lcd_digit48( 1, 1,(int)temp / 10);
//	else
//		lcd_null48(1, 1);
//
//	lcd_digit48(27, 1, (int)temp % 10);
//
//	lcd1100_gotoxy((FONT48_WIDTH)+4, 6); //dot
//	lcd1100_write(lcd1100_DATA, 0x78);
//	lcd1100_write(lcd1100_DATA, 0x78);
//	lcd1100_write(lcd1100_DATA, 0x78);
//	lcd1100_write(lcd1100_DATA, 0x78);
//
//	lcd_digit16((FONT48_WIDTH) + 5, 5,  (int)(temp*10.0) % 10); 
//
//	//lcd1100_gotoxy((FONT48_WIDTH) + 6 + FONT16_WIDTH + 6, 6);
//}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void StartPwmDemo(void const * argument)
{

  for (;;) {
 #if 0   
    for (Triac_angle=3000;Triac_angle < 9800;Triac_angle+=10) 
    {
      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width);
      osDelay(5);
    }

    osDelay(3000);
    
    for (Triac_angle=9800;Triac_angle > 3000;Triac_angle-=10) 
    {
      //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_width);
      osDelay(5);
    }
    
osDelay(3000);
#endif

if (Standby == 0) 
{
pid_compute(pid);
}

Triac_angle = map ((int)Power, 0, 100, 3000, 9800);

osDelay(500);

}
}

void InterfaceTask (void const * argument)
{ 
   
  for (;;)
  {
        eventFuncButt = osMailGet(xMailFuncButt, 1000);
    
    if( eventFuncButt.status == osEventMail )
    {
      functButtTmp = *((StrFunctButt *)eventFuncButt.value.p);
      
       if (functButtTmp.typeNo[0] == PRESS_HOLD) {
               switch (functButtTmp.buttNo[0]) 
      {
        case BUT_MINUS:
          
          Standby = 1;
          Power = 0;
          break;
          
        case BUT_PLUS:
          Standby = 0;
          break;
      }
       }
       
       if (functButtTmp.typeNo[0] == PRESS_NONE) {
         switch (functButtTmp.buttNo[0]) 
      {
        case BUT_MINUS:
          
          
          break;
      }
         
       }
       
     if (functButtTmp.typeNo[0] == PRESS_SIMPLE) {
         switch (functButtTmp.buttNo[0]) 
      {
        case BUT_MINUS:
          
          Setpoint -= 0.5f;
          break;
          
        case BUT_PLUS:
          
          Setpoint += 0.5f;
          break;
      }        
       }

     if (functButtTmp.typeNo[0] == PRESS_DOUBLE) {
         switch (functButtTmp.buttNo[0]) 
      {
        case BUT_MINUS:
          Setpoint -= 2.0f;
          break;
          
        case BUT_PLUS:
          Setpoint += 2.0f;
          break;
      }        
       }
       
       
    } else if( eventFuncButt.status == osErrorParameter ) 
      osDelay(2000);
    
  }
}

void DisplayTask(void const * argument)
{
lcd1100_pin_init();
lcd1100_init();
lcd1100_clear();

//update_spd();
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 100);

lcd1100_fill_image(troll);
lcd1100_gotoxy(12,7);
lcd1100_puts("v1.0");
osDelay(2500);
lcd1100_clear();

lcd1100_gotoxy(1,1);
lcd1100_puts_big(4,8,"00");
lcd1100_gotoxy(9,4);
lcd1100_puts_big(2,4,".00");

//lcd1100_gotoxy(11,2);
//lcd1100_puts("HEAT");

lcd1100_gotoxy(11,0);
sprintf(str, "%.1f\r\n", Setpoint);
lcd1100_puts(str);

lcd1100_gotoxy(0,0);
lcd1100_puts("[       ]");

Standby = 0;

  for (;;)
  {
                        lcd1100_clear();
                        sprintf(str, "%02d\r\n", (int)Temperature);
                        lcd1100_gotoxy(1,1);
                        lcd1100_puts_big(4,8,str);
                        
                        sprintf(str, ".%02d\r\n", ((int)(Temperature*100)%100));
                        lcd1100_gotoxy(9,4);
                        lcd1100_puts_big(2,4, str);
                        
                        lcd1100_gotoxy(0,0);
                        if (Power < 10 )                lcd1100_puts("[       ]");
                        if (Power >= 10 && Power < 20 ) lcd1100_puts("[=      ]");
                        if (Power >= 20 && Power < 30 ) lcd1100_puts("[==     ]");
                        if (Power >= 30 && Power < 40 ) lcd1100_puts("[===    ]");
                        if (Power >= 40 && Power < 50 ) lcd1100_puts("[====   ]");
                        if (Power >= 50 && Power < 70 ) lcd1100_puts("[=====  ]");
                        if (Power >= 70 && Power < 90 ) lcd1100_puts("[====== ]");
                        if (Power >= 90 && Power <= 100)lcd1100_puts("[=======]");
                        
                        if (Standby == 1) {
                          lcd1100_gotoxy(11,2);
                          lcd1100_puts("OFF ");
                        } 
                        else 
                        {        
                          lcd1100_gotoxy(11,2);
                          lcd1100_puts("HEAT");
                          
                          lcd1100_gotoxy(11,0);
                          sprintf(str, "%.1f\r\n", Setpoint);
                          lcd1100_puts(str);
                        }
                        
                        osDelay(500);
                        
  }
  
  
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  
  //EE_Reads(0,1,Setpoint);


status = OW_Scan((uint8_t*)str,2);

// Create PID controllers, set gains
	pid = pid_create(&pidctrl, &Temperature, &Power, &Setpoint, 25, 5, 10);
	// Set output limits
	pid_limits(pid, 0, 100);
	// Turn on PID
	pid_auto(pid);

  /* Infinite loop */
  for(;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    
    OW_Send(OW_SEND_RESET, (uint8_t*)"\xcc\x44", 2, NULL, NULL, OW_NO_READ);
    
    _ow_power_pin();
    
    osDelay(2000);
   
    //if (br_pulse == 0) br_pulse = 100; else br_pulse = 0;
    
    //__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, br_pulse);
    
    _ow_tx_pin();
      
    status=OW_Send(OW_SEND_RESET,(uint8_t*) "\xcc\xbe\xff\xff", 4, buf,2, 2);
		
                if (status==OW_OK)
		{
			t=(uint16_t*)buf;
                        
                        temp=*t*0.0625;

                        if ((temp != 85.0f) && (temp < 125.0f) && (temp > -55.0f)) //85.0 result is not ready in sensor
                        {
                          Temperature = temp;
                          //Standby = 0;
                        }

                      
		}
                
    
    
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
