
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
#include "eeprom.h"
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
__IO uint32_t Triac_angle = 3000;
 float Temperature = 255.0f;
 float Power = 0;
 float Setpoint = 20.0;
 uint8_t nErrors = 0xFF;
 uint8_t Standby = 1;

uint16_t eeSetpoint;
uint16_t eeStandby;

uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5555, 0x6666, 0x7777};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue = 0;


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
  HAL_FLASH_Unlock();
  /* EEPROM Init */
  EE_Init();
  
  EE_ReadVariable(VirtAddVarTab[0], &eeSetpoint);
  EE_ReadVariable(VirtAddVarTab[1], &eeStandby);
  
  if (eeSetpoint > 15 && eeSetpoint < 26)
  {
    Setpoint = eeSetpoint;
  } else
  {
    EE_WriteVariable(VirtAddVarTab[0], 20);
  }
  
  if (eeStandby > 1)
  {
    EE_WriteVariable(VirtAddVarTab[1], 0);
  } else
  {
    //Standby = eeStandby;
  }
  
  
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
  
  osThreadDef(displayTask, DisplayTask, osPriorityLow, 0, 256);
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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
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
const uint8_t troll[] = 
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
char str[16];

union {
   uint16_t integer;
   uint8_t  raw[2];
} data;

int s,m,e;

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

if (Standby == 0) 
{
pid_compute(pid);
} else 
{
  Power = 0;
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
          
          eeStandby = Standby;
          
          EE_WriteVariable(VirtAddVarTab[1], eeStandby);
          
          break;
          
        case BUT_PLUS:
          Standby = 0;
          
          eeStandby = Standby;
          
          EE_WriteVariable(VirtAddVarTab[1], eeStandby);
                   
          break;
      }
       }
       
       if (functButtTmp.typeNo[0] == PRESS_NONE) {
         switch (functButtTmp.buttNo[0]) 
      {
        case BUT_MINUS:  
          break;
           case BUT_PLUS:  
          break;
      }
         
       }
       
     if (functButtTmp.typeNo[0] == PRESS_SIMPLE) {
         switch (functButtTmp.buttNo[0]) 
      {
        case BUT_MINUS:
          
          Setpoint -= 1.0f;
          
          if (Setpoint < 10 ) Setpoint = 10;
          
          eeSetpoint = (int16_t)Setpoint;
          
          EE_WriteVariable(VirtAddVarTab[0], eeSetpoint);
          
          break;
          
        case BUT_PLUS:
          
          Setpoint += 1.0f;
          
          if (Setpoint > 26 ) Setpoint = 26;
          
          eeSetpoint = (int16_t)Setpoint;
          
          if (EE_WriteVariable(VirtAddVarTab[0], eeSetpoint) == PAGE_FULL) {
            //EE_Format();
          }
          
          break;
      }        
       }

     if (functButtTmp.typeNo[0] == PRESS_DOUBLE) {
         switch (functButtTmp.buttNo[0]) 
      {
        case BUT_MINUS:
          Setpoint -= 2.0f;
          
          if (Setpoint < 10 ) Setpoint = 10;
          
          eeSetpoint = (int16_t)Setpoint;
          
          if (EE_WriteVariable(VirtAddVarTab[0], eeSetpoint) == PAGE_FULL) {
            //EE_Format();
          }
          
          break;
          
        case BUT_PLUS:
          Setpoint += 2.0f;
          
          if (Setpoint > 26 ) Setpoint = 26;
          
          eeSetpoint = (int16_t)Setpoint;
          
          if (EE_WriteVariable(VirtAddVarTab[0], eeSetpoint) == PAGE_FULL) {
            //EE_Format();
          }
          
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

lcd1100_fill_image((uint8_t*)troll);
lcd1100_gotoxy(11,7);
lcd1100_puts("v.1.2");
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

Standby = eeStandby;

  for (;;)
  {
    //lcd1100_clear();
        
    if (Temperature <= 99.0f  &&  Temperature >= -9.0f) {
                        sprintf(str, "%02d\r\n", (int)Temperature);
                        lcd1100_gotoxy(1,1);
                        lcd1100_puts_big(4,8,str);
                        
                        sprintf(str, ".%02d\r\n", ((int)(Temperature*100)%100));
                        lcd1100_gotoxy(9,4);
                        lcd1100_puts_big(2,4, str);
    }
    
    else
    {
      lcd1100_gotoxy(1,1);
      lcd1100_puts_big(4,8,"--");
      lcd1100_gotoxy(9,4);
      lcd1100_puts_big(2,4,".--");
    }
                        lcd1100_gotoxy(0,0);
                             if (Power >=  0 && Power < 10)  lcd1100_puts("[       ]");
                        else if (Power >= 10 && Power < 20 ) lcd1100_puts("[=      ]");
                        else if (Power >= 20 && Power < 30 ) lcd1100_puts("[==     ]");
                        else if (Power >= 30 && Power < 40 ) lcd1100_puts("[===    ]");
                        else if (Power >= 40 && Power < 50 ) lcd1100_puts("[====   ]");
                        else if (Power >= 50 && Power < 70 ) lcd1100_puts("[=====  ]");
                        else if (Power >= 70 && Power < 90 ) lcd1100_puts("[====== ]");
                        else if (Power >= 90 && Power <= 100)lcd1100_puts("[=======]");                        
                        
                        if (Standby == 1) {
                          lcd1100_gotoxy(11,2);
                          lcd1100_puts("OFF ");
                        } 
                        else 
                        {        
                          lcd1100_gotoxy(11,2);
                          lcd1100_puts("HEAT");                         
                        }
                        
                          lcd1100_gotoxy(11,0);
                          sprintf(str, "%.1f\r\n", Setpoint);
                          lcd1100_puts(str);
                        
                        osDelay(500);
                        
  }
  
  
}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  
  //Load_User_Data();
  
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
      
    status=OW_Send(OW_SEND_RESET,(uint8_t*) "\xcc\xbe\xff\xff", 4, data.raw,2, 2);
		
                if (status==OW_OK)
		{                       
                        temp = data.integer*0.0625;

                        if (temp != 85.0f && temp <= 99.0f  &&  temp >= -9.0f) //85.0 result is not ready in sensor
                        {
                          nErrors = 0;
                          Temperature = temp;
                        } else
                        {
                          nErrors ++;
                        }
                      
		}
                else
                {
                  nErrors ++;
                }
                
                if (nErrors > 10) 
                {
                Standby = 1;
                Power = 0;
                Temperature = 255.0f;
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
