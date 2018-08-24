#include <string.h>
#include "driverButton.h"
#include "cmsis_os.h"

osMailQId xMailFuncButt; 
osMessageQId xQueueButton;
osEvent eventButton;  
StrFunctButt functButt;
extern osThreadId GUI_ThreadId;

GPIO_TypeDef* GPIO_PORT[BUT_AMOUNT] = {
                                       BUT_PLUS_GPIO_Port,
                                       BUT_MINUS_GPIO_Port};

const uint16_t GPIO_PIN[BUT_AMOUNT] = {
                                       BUT_PLUS_Pin,
                                       BUT_MINUS_Pin};
#ifdef EVENT
const IRQn_Type GPIO_IRQn[BUT_AMOUNT] = {
                                       BUTT_LFT_EXTI_IRQn,
                                       BUTT_UP_EXTI_IRQn};
#endif

osEvent eventFuncButt; 
StrFunctButt functButtTmp;

/**
  * @brief  Драйвер клавиатруы
  * @note   None
  * @param  None
  * @retval None
  */
void vDriverButton( void const *pvParameters )
{
  short i0;
  int8_t iPressButton[BUT_AMOUNT];
  BOOL isPressButton[BUT_AMOUNT];
  uint8_t indPressButt[BUT_AMOUNT];
  memset(iPressButton, 0, BUT_AMOUNT);
  memset(isPressButton, FALSE, BUT_AMOUNT);
  memset(indPressButt, 0, BUT_AMOUNT);
  STEP_BUTTON iState = WAIT_PRESS;
  BOOL _isPressButton = FALSE;
  uint16_t iTime = 0;
  uint16_t iPressButtons = 0;

  for (;;)
  {
    osDelay(PERIOD_BUTTON);      
    
    /* Определение состояния кнопок(дребезг): нажата/нет*/
    for (i0 = 0; i0 < BUT_AMOUNT; i0++)
    {
      if (HAL_GPIO_ReadPin(GPIO_PORT[i0], GPIO_PIN[i0]))
        iPressButton[i0]--;
      else
        iPressButton[i0]++;      
      if (iPressButton[i0] < (-N_CALC_BUTTON))
      {
        iPressButton[i0] = -N_CALC_BUTTON;
        isPressButton[i0] = FALSE;
      } else
      if (iPressButton[i0] > (N_CALC_BUTTON))
      {
        iPressButton[i0] = N_CALC_BUTTON; 
        isPressButton[i0] = TRUE;
      }
    }
    
    iPressButtons = 0;
    for (i0 = 0; i0 < BUT_AMOUNT; i0++)
      if (isPressButton[i0])
        indPressButt[iPressButtons++] = i0;
    
    /* Нажатие 2-х и более кнопок одновременно*/
    if (iPressButtons >= 2)
    {
      iState = WAIT_DOWN;
      functButt.nPressButt = iPressButtons;
      for (i0 = 0; i0 < iPressButtons; i0++)
      {
        functButt.buttNo[i0] = indPressButt[i0];
        functButt.typeNo[i0] = PRESS_COMB;
      }
      osMailPut(xMailFuncButt, &functButt);
    } else
      _isPressButton = isPressButton[indPressButt[0]];
    
    switch (iState)
    {
    case CHECK_START:
      if (_isPressButton)
        iState = CHECK_HOLD_LONG;
      break;
    case CHECK_HOLD_LONG:
      iTime += PERIOD_BUTTON;
       
      /* Удержание клавиши*/
      if (iTime > BUT_HOLD_TIME)
      {
        iState = WAIT_DOWN;
        functButt.nPressButt = 1;
        functButt.buttNo[0] = indPressButt[0];
        functButt.typeNo[0] = PRESS_HOLD;
        osMailPut(xMailFuncButt, &functButt);
        break;
      }
      
      if (!_isPressButton)
      {
        /* Долгое нажатие*/
        if (iTime > BUT_LONG_TIME)
        {        
          iState = WAIT_PRESS;
          functButt.nPressButt = 1;
          functButt.buttNo[0] = indPressButt[0];
          functButt.typeNo[0] = PRESS_LONG;
          osMailPut(xMailFuncButt, &functButt);
          break;
        }
        iTime = 0;
        iState = CKECK_DOUBLE_SIMPLE;
      }
      break;
    case CKECK_DOUBLE_SIMPLE:
      iTime += PERIOD_BUTTON;
          
      /* Двойное нажатие*/
      if (_isPressButton)
      {
        iState = WAIT_DOWN;
        functButt.nPressButt = 1;
        functButt.buttNo[0] = indPressButt[0];
        functButt.typeNo[0] = PRESS_DOUBLE;
        osMailPut(xMailFuncButt, &functButt);
        break;
      }
      
      /* Простое нажатие*/
      if (iTime > BUT_DOUBLE_TIME)
      {
        iState = WAIT_PRESS;
        functButt.nPressButt = 1;
        functButt.buttNo[0] = indPressButt[0];
        functButt.typeNo[0] = PRESS_SIMPLE;
        osMailPut(xMailFuncButt, &functButt);
        break;
      }
      break;
    case WAIT_DOWN:
      /* Ожидание отжатия клавиши*/
      if (!_isPressButton)
      {
        iState = WAIT_PRESS; 
        functButt.nPressButt = 0;
        functButt.typeNo[0] = PRESS_NONE;
        osMailPut(xMailFuncButt, &functButt);
      }
      break;
    case WAIT_PRESS:
#ifdef EVENT
      /* Ожидание событие для возобно-я задачи*/
      for (int i0 = 0; i0 < BUT_AMOUNT; i0++)         
        HAL_NVIC_EnableIRQ(GPIO_IRQn[i0]);
      
      eventButton = osMessageGet( xQueueButton, osWaitForever );   
      if( eventButton.status == osEventMessage )
      {
        iTime = 0;
        iState = CHECK_START;
        for (int i0 = 0; i0 < BUT_AMOUNT; i0++)         
          HAL_NVIC_DisableIRQ(GPIO_IRQn[i0]);
        
        while (uxQueueMessagesWaiting(xQueueButton) != 0)
          osMessageGet( xQueueButton, osWaitForever ); 
      }
#else
      iTime = 0;
      iState = CHECK_START;
#endif
      break;
    }
  }
}
 

void initButton()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  for (int i0 = 0; i0 < BUT_AMOUNT; i0++)
  {
    GPIO_InitStruct.Pin = GPIO_PIN[i0];
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIO_PORT[i0], &GPIO_InitStruct);
    
    //HAL_NVIC_SetPriority(GPIO_IRQn[i0], 15, 15);
    //HAL_NVIC_EnableIRQ(GPIO_IRQn[i0]);
  }
  
  osMessageQDef(queueButton, 1, uint8_t);
  xQueueButton = osMessageCreate (osMessageQ(queueButton), NULL);

  osMailQDef(mailFuncButt, 1, StrFunctButt);
  xMailFuncButt = osMailCreate (osMailQ(mailFuncButt), NULL);
  
  /* Задача, драйвер клавиатуры */
  osThreadDef(DriverButton, vDriverButton, osPriorityLow, 1, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(DriverButton), NULL);
  
}