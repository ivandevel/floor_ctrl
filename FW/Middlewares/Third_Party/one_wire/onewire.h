/*
 * onewire.h
 *
 *  Version 1.0.1
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

// для разных процессоров потребуется проверить функцию OW_Init
// на предмет расположения ножек USART
#include "stm32f0xx_hal.h"
#include "dma_ctrl.h"

// выбираем, на каком USART находится 1-wire
#define OW_USART2


#ifdef OW_USART1

#undef OW_USART2
#undef OW_USART3
#undef OW_USART4

#define OW_USART                USART1
#define OW_DMA			DMA1
#define OW_DMA_CH_RX            DMA2_Stream5
#define OW_DMA_CH_TX            DMA2_Stream7
#define OW_DMA_FLAG_RX          DMA_IT_TCIF5
#define OW_DMA_FLAG_TX          DMA_IT_TCIF7

#endif


#ifdef OW_USART2

#undef OW_USART1
#undef OW_USART3
#undef OW_USART4

#define OW_USART                USART2
#define OW_DMA			DMA1

#define OW_DMA_CH_RX            DMA1_Channel5
#define OW_DMA_CH_TX            DMA1_Channel4

#define OW_DMA_FLAG_RX          DMA_IT_TCIF5
#define OW_DMA_FLAG_TX          DMA_IT_TCIF7

#define DQ_Pin GPIO_PIN_2
#define DQ_GPIO_Port GPIOA

#endif

// если нужно отдавать тики FreeRTOS, то раскомментировать
#define OW_GIVE_TICK_RTOS

// первый параметр функции OW_Send
#define OW_SEND_RESET		1
#define OW_NO_RESET		2

// статус возврата функций
#define OW_OK		1
#define OW_ERROR	2
#define OW_NO_DEVICE	3

#define OW_NO_READ		0xff

#define OW_READ_SLOT	0xff

uint8_t OW_Init(void);
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
void _ow_power_pin(void);
void _ow_tx_pin(void);
uint8_t OW_Scan(uint8_t *buf, uint8_t num);

#endif /* ONEWIRE_H_ */
