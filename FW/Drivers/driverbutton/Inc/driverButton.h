#ifndef _DRIVER_BUTTON
#define _DRIVER_BUTTON

//#include "stm32f4xx_hal.h"
//#include "stm32f2xx_hal.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
//#include "typeProject.h"

//#define EVENT

#define N_CALC_BUTTON   2       /* Значение счетчика для определения однозначности состояния кнопки*/
#define PERIOD_BUTTON   1       /**/

#define BUT_HOLD_TIME   2000 /* Уденржание */    
#define BUT_LONG_TIME   500  /* Долгое нажатие */
#define BUT_DOUBLE_TIME 150    
  
#define TRUE 1
#define FALSE 0

typedef uint8_t BOOL;

/* Состояние кнопки
*/
typedef enum { 
  PRESS_NONE = 0,       /* Нет нажатия*/
  PRESS_SIMPLE,         /* Простое нажатие*/
  PRESS_DOUBLE,         /* Двойное нажатие*/
  PRESS_LONG,           /* Долгое нажатие*/
  PRESS_HOLD,           /* Удержание*/
  PRESS_COMB            /* Комбо нажатие, т.е. нажатие 2-х и более кнопок*/
} STATE_BUTTON;

/* Индексы кнопок в массиве
*/
typedef enum {
  BUT_PLUS=0,
  BUT_MINUS,
  BUT_AMOUNT
} IND_BUTT; 

/* Индексы кнопок в массиве
*/
typedef enum {
  CHECK_START = 0,
  CHECK_HOLD_LONG,
  CKECK_DOUBLE_SIMPLE,
  WAIT_DOWN,
  WAIT_PRESS,
} STEP_BUTTON;

/* Функциональные сообщение клавиатуры
*/
typedef struct 
{
  int nPressButt;               /* Кол-во нажатых кнопок*/
  int buttNo[BUT_AMOUNT];       /* Номера нажатых кнопок*/
  int typeNo[BUT_AMOUNT];       /* Типы нажатых кнопок*/
} StrFunctButt;

extern GPIO_TypeDef* GPIO_PORT[BUT_AMOUNT];
extern const uint16_t GPIO_PIN[BUT_AMOUNT];
extern const IRQn_Type GPIO_IRQn[BUT_AMOUNT];
extern osMessageQId xQueueButton;
extern osMailQId xMailFuncButt;

extern osEvent eventFuncButt; 
extern StrFunctButt functButtTmp;

void initButton();
void vDriverButton( void const *pvParameters );

#endif
