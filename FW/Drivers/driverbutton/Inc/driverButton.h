#ifndef _DRIVER_BUTTON
#define _DRIVER_BUTTON

//#include "stm32f4xx_hal.h"
//#include "stm32f2xx_hal.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
//#include "typeProject.h"

//#define EVENT

#define N_CALC_BUTTON   2       /* �������� �������� ��� ����������� ������������� ��������� ������*/
#define PERIOD_BUTTON   1       /**/

#define BUT_HOLD_TIME   2000 /* ���������� */    
#define BUT_LONG_TIME   500  /* ������ ������� */
#define BUT_DOUBLE_TIME 150    
  
#define TRUE 1
#define FALSE 0

typedef uint8_t BOOL;

/* ��������� ������
*/
typedef enum { 
  PRESS_NONE = 0,       /* ��� �������*/
  PRESS_SIMPLE,         /* ������� �������*/
  PRESS_DOUBLE,         /* ������� �������*/
  PRESS_LONG,           /* ������ �������*/
  PRESS_HOLD,           /* ���������*/
  PRESS_COMB            /* ����� �������, �.�. ������� 2-� � ����� ������*/
} STATE_BUTTON;

/* ������� ������ � �������
*/
typedef enum {
  BUT_PLUS=0,
  BUT_MINUS,
  BUT_AMOUNT
} IND_BUTT; 

/* ������� ������ � �������
*/
typedef enum {
  CHECK_START = 0,
  CHECK_HOLD_LONG,
  CKECK_DOUBLE_SIMPLE,
  WAIT_DOWN,
  WAIT_PRESS,
} STEP_BUTTON;

/* �������������� ��������� ����������
*/
typedef struct 
{
  int nPressButt;               /* ���-�� ������� ������*/
  int buttNo[BUT_AMOUNT];       /* ������ ������� ������*/
  int typeNo[BUT_AMOUNT];       /* ���� ������� ������*/
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
