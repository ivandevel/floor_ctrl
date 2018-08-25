/*
 * onewire.c
 *
 *  Created on: 13.02.2012
 *      Author: di
 */

#include "onewire.h"

#ifdef OW_GIVE_TICK_RTOS
#include "cmsis_os.h"
#endif

static UART_HandleTypeDef _huart;//��������� ���������� ���

//-----------------------------------------------------------------------------
// ������� �������������� UART_OW
//-----------------------------------------------------------------------------
 static void _ow_usart_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_USART2_CLK_ENABLE();
  
	/**USART1 GPIO Configuration    
	PA2     ------> USART1_TX 
	*/
	GPIO_InitStruct.Pin = DQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(DQ_GPIO_Port, &GPIO_InitStruct);

        HAL_GPIO_WritePin(DQ_GPIO_Port, DQ_Pin, GPIO_PIN_SET);
}

//-----------------------------------------------------------------------------
// ������� �������������� UART_OW  
//-----------------------------------------------------------------------------
  void _ow_power_pin(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/**USART1 GPIO Configuration    
	PA2     ------> USART1_TX 
	*/
	GPIO_InitStruct.Pin = DQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(DQ_GPIO_Port, &GPIO_InitStruct);
        
}

//-----------------------------------------------------------------------------
// ������� �������������� UART_OW  
//-----------------------------------------------------------------------------
  void _ow_tx_pin(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
  
	/**USART1 GPIO Configuration    
	PA2     ------> USART1_TX 
	*/
	GPIO_InitStruct.Pin = DQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
	HAL_GPIO_Init(DQ_GPIO_Port, &GPIO_InitStruct);
}

//-----------------------------------------------------------------------------
// ������� ������������� UART_OW � ��������� ���������
//-----------------------------------------------------------------------------
 static void _ow_usart_set_speed(uint32_t speed)
{
	_huart.Instance = OW_USART;
	_huart.Init.BaudRate = speed;
	_huart.Init.WordLength = UART_WORDLENGTH_8B;
	_huart.Init.StopBits = UART_STOPBITS_1;
	_huart.Init.Parity = UART_PARITY_NONE;
	_huart.Init.Mode = UART_MODE_TX_RX;
	_huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	_huart.Init.OverSampling = UART_OVERSAMPLING_16;
	_huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_ENABLE;
	_huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	HAL_HalfDuplex_Init(&_huart);	
}

//-----------------------------------------------------------------------------
// ������� ������� �������� �� UART 1 �����
//-----------------------------------------------------------------------------
 static void _ow_uart_byte_send(uint8_t data)
{
	HAL_UART_Transmit(&_huart, &data, 1, 100);
}
//-----------------------------------------------------------------------------
// ������� ������� ��������/������ �� UART ����� DMA
//-----------------------------------------------------------------------------
 static uint32_t _ow_uart_dma_send_recv(uint8_t* data, uint32_t cnt)
{
	//uint32_t i=0;
	uint32_t status=OW_OK;
	// DMA �� ������
	uart_dma_rx_link(OW_USART,OW_DMA_CH_RX,data,cnt);	
	// DMA �� ������
	uart_dma_tx_link(OW_USART,OW_DMA_CH_TX);
	//�������� ��� �����
	SET_BIT(OW_DMA->IFCR,DMA_IFCR_CTCIF4);//��������� ���� ��������
	SET_BIT(OW_DMA->IFCR,DMA_IFCR_CTCIF5);//��������� ���� ������	
	
	uart_dma_tx(OW_DMA_CH_TX, data, cnt);
		
	// ����, ���� �� ������ 8 ����
		while (READ_BIT(OW_DMA->ISR, DMA_IFCR_CTCIF5)==0)
		{
#ifdef OW_GIVE_TICK_RTOS
			taskYIELD();
#endif
#ifndef OW_GIVE_TICK_RTOS
			HAL_Delay(1);
			i++;
			if(i==1000)
			{
				status=OW_ERROR;
				break;
			}
#endif				
		}
		// ��������� DMA
		//����� ����������� ����� �������� ��� ��������� �����
		SET_BIT(OW_DMA->IFCR,DMA_IFCR_CGIF4);
		SET_BIT(OW_DMA->IFCR,DMA_IFCR_CHTIF4);
		SET_BIT(OW_DMA->IFCR,DMA_IFCR_CTCIF4);
		
		SET_BIT(OW_DMA->IFCR,DMA_IFCR_CGIF5);
		SET_BIT(OW_DMA->IFCR,DMA_IFCR_CHTIF5);	
		SET_BIT(OW_DMA->IFCR,DMA_IFCR_CTCIF5);
		
		CLEAR_BIT(OW_DMA_CH_TX->CCR,DMA_CCR_EN);
		CLEAR_BIT(OW_DMA_CH_RX->CCR,DMA_CCR_EN);

		return status;
}
//-----------------------------------------------------------------------------
// ������� ������� ������ �� UART 1 �����
//-----------------------------------------------------------------------------
 static uint8_t _ow_uart_byte_recv(void)
{
	uint8_t data;
	HAL_UART_Receive(&_huart, &data, 1, 100);
	return data;
}

// ����� ��� ������/�������� �� 1-wire
uint8_t ow_buf[8];

#define OW_0	  0x00
#define OW_1	  0xff
#define OW_R_1	  0xff

//-----------------------------------------------------------------------------
// ������� ����������� ���� ���� � ������, ��� �������� ����� USART
// ow_byte - ����, ������� ���� �������������
// ow_bits - ������ �� �����, �������� �� ����� 8 ����
//-----------------------------------------------------------------------------
void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// �������� �������������� - �� ����, ��� �������� ����� USART ����� ���������� ����
// ow_bits - ������ �� �����, �������� �� ����� 8 ����
//-----------------------------------------------------------------------------
uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

//-----------------------------------------------------------------------------
// �������������� USART
//-----------------------------------------------------------------------------
uint8_t OW_Init(void) 
{
	_ow_usart_init();
	return OW_OK;
}
//-----------------------------------------------------------------------------
// ������������ ����� � �������� �� ������� ��������� �� ����
//-----------------------------------------------------------------------------
uint8_t OW_Reset(void) {
	uint8_t ow_presence;
	_ow_usart_set_speed(9600);
	// ���������� 0xf0 �� �������� 9600
	uint8_t data=0xf0;
	_ow_uart_byte_send(data);
	ow_presence=_ow_uart_byte_recv();
	_ow_usart_set_speed(115200);

	if (ow_presence != 0xf0) {
		return OW_OK;
	}

	return OW_NO_DEVICE;
}


//-----------------------------------------------------------------------------
// ��������� ������� � ����� 1-wire
// sendReset - �������� RESET � ������ �������.
// 		OW_SEND_RESET ��� OW_NO_RESET
// command - ������ ����, ���������� � ����. ���� ����� ������ - ���������� OW_READ_SLOTH
// cLen - ����� ������ ������, ������� ���� ��������� � ����
// data - ���� ��������� ������, �� ������ �� ����� ��� ������
// dLen - ����� ������ ��� ������. ����������� �� ����� ���� �����
// readStart - � ������ ������� �������� �������� ������ (���������� � 0)
//		����� ������� OW_NO_READ, ����� ����� �� �������� data � dLen
//-----------------------------------------------------------------------------
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) 
{
	volatile uint32_t i=0;
	// ���� ��������� ����� - ���������� � ��������� �� ������� ���������
	if (sendReset == OW_SEND_RESET) 
	{
		if (OW_Reset() == OW_NO_DEVICE) return OW_NO_DEVICE;
	}
	while (cLen > 0) 
	{

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;
		
		if (_ow_uart_dma_send_recv(ow_buf, 8) == OW_ERROR) return OW_ERROR;
		
		// ���� ����������� ������ ����-�� ����� - ������� �� � �����
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		} else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}

//-----------------------------------------------------------------------------
// ������ ������� ������������ ������������ ���� 1-wire � ���������� ���������
//   ID ��������� � ������ buf, �� 8 ���� �� ������ ����������.
// ���������� num ������������ ���������� ��������� ���������, ����� �� �����������
// �����.
//-----------------------------------------------------------------------------
uint8_t OW_Scan(uint8_t *buf, uint8_t num) {

	uint8_t found = 0;
	uint8_t *lastDevice;
	uint8_t *curDevice = buf;
	uint8_t numBit, lastCollision, currentCollision, currentSelection;

	lastCollision = 0;
	while (found < num) {
		numBit = 1;
		currentCollision = 0;

		// �������� ������� �� ����� ���������
		OW_Send(OW_SEND_RESET, (uint8_t*)"\xf0", 1, 0, 0, OW_NO_READ);

		for (numBit = 1; numBit <= 64; numBit++) {
			// ������ ��� ����. �������� � ���������������
			OW_toBits(OW_READ_SLOT, ow_buf);
			
                        if (_ow_uart_dma_send_recv(ow_buf, 2) == OW_ERROR) return OW_ERROR;

			if (ow_buf[0] == OW_R_1) {
				if (ow_buf[1] == OW_R_1) {
					// ��� �������, ���-�� ���������� � ����������� �����
					return found;
				} else {
					// 10 - �� ������ ����� ������ 1
					currentSelection = 1;
				}
			} else {
				if (ow_buf[1] == OW_R_1) {
					// 01 - �� ������ ����� ������ 0
					currentSelection = 0;
				} else {
					// 00 - ��������
					if (numBit < lastCollision) {
						// ���� �� ������, �� ����� �� ��������
						if (lastDevice[(numBit - 1) >> 3]
								& 1 << ((numBit - 1) & 0x07)) {
							// (numBit-1)>>3 - ����� �����
							// (numBit-1)&0x07 - ����� ���� � �����
							currentSelection = 1;

							// ���� ����� �� ������ �����, ���������� ����� ����
							if (currentCollision < numBit) {
								currentCollision = numBit;
							}
						} else {
							currentSelection = 0;
						}
					} else {
						if (numBit == lastCollision) {
							currentSelection = 0;
						} else {
							// ���� �� ������ �����
							currentSelection = 1;

							// ���� ����� �� ������ �����, ���������� ����� ����
							if (currentCollision < numBit) {
								currentCollision = numBit;
							}
						}
					}
				}
			}

			if (currentSelection == 1) {
				curDevice[(numBit - 1) >> 3] |= 1 << ((numBit - 1) & 0x07);
				OW_toBits(0x01, ow_buf);
			} else {
				curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
				OW_toBits(0x00, ow_buf);
			}
			
                        if (_ow_uart_dma_send_recv(ow_buf, 1) == OW_ERROR) return OW_ERROR;
		}
		found++;
		lastDevice = curDevice;
		curDevice += 8;
		if (currentCollision == 0)
			return found;

		lastCollision = currentCollision;
	}

	return found;
}

