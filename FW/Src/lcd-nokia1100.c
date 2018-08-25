//************************************************************************//
//	LCD NOKIA1100
//	Alex_EXE 
//	http://alex-exe.ru/category/radio/stm32/lcd-nokia1100-stm32
//	http://www.sunbizhosting.com/~spiral/1100/
//************************************************************************//

/*
	PIN configuration: Nokia 3510 LCD PCF8814
	
	to know the pins, view the screen from front, pin 1 is on right side
	read : http://www.circuitvalley.com/2011/09/nokia-1100-lcd-interfacing-with.html
	
		|----------------|
		|  screen        |
		|    front       |
		|________________|
		 | | | | | | | |
		 1 2 3 4 5 6 7 8
		 
		 PIN 1: XRES
		 PIN 2: XCS
		 PIN 3: GND
		 PIN 4: MOSI
		 PIN 5: SCLK
		 PIN 6: VDDI
		 PIN 7: VDD 
		 PIN 8: Vcap

	 WE shorted PIN 6, 7
*/

#include "lcd-nokia1100.h"
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

#define LCD_NOP 0xE3
#define LCD_MODE 0xA0
#define LCD_VOB_MSB 0x20
#define LCD_VOB_LSB 0x80
#define LCD_CHARGE_PUMP_ON 0x2F
#define LCD_RAM_ADDR_MODE 0xAA
#define LCD_CHANGE_ROW_LSB 0x00
#define LCD_CHANGE_ROW_MSB 0x10
#define LCD_CHANGE_COL 0xB0
#define LCD_MIRROR_Y 0xC0
#define LCD_MIRROR_X 0xA0
#define LCD_EXT_OSC 0x3A
#define LCD_SOFT_RESET 0xE2
//#define LCD_DATA_DISPLAY_LEN , read page no 27
//#define LCD_FACTORY_DEFAULT ,read page no 27 of datasheet
//#define LCD_REFRESH_RATE ,read page 27,29 of datasheet

#define ON 0x01
#define OFF 0x00
#define ALL 0x04
#define INVERT 0x06
#define DISPLAY 0x0E

/*----------------------------------------------------------------------------------------------*/
#define CHAR_WIDTH 6
#define CHAR_HEIGHT 8


unsigned char lcd1100_X, lcd1100_Y;//	Две глобальные переменные расположения курсора

//************************************************************************//
GPIO_InitTypeDef lcd1100_GPIO_InitStructure;	//	структура для инициализации выводов дисплея

//	Задержка для дисплея. Можно подставить местную функцию задержки
void delay_lcd1100(unsigned long p)
{
osDelay(p);
	//p=p*1000;
	//while(p-->0){};        
}

//	инициализация выводов используемым дисплеем
void lcd1100_pin_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  
  HAL_GPIO_WritePin(GPIOA, LCD_CLK_Pin|LCD_SDA_Pin|LCD_CS_Pin|LCD_RST_Pin, GPIO_PIN_SET);
  
  lcd1100_RCC_Enable;
  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = lcd1100_pin_SCLK | lcd1100_pin_SDA | lcd1100_pin_CS | lcd1100_pin_RST;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(lcd1100_port, &GPIO_InitStruct);
        
}
//************************************************************************//

//	Передать данные дисплею
//	Input : cd - команда или данные, с - байт данных или команды
void lcd1100_write(char cd, unsigned char c)
{
	char i;
        //asm("nop");
	lcd1100_CS_off;
        //asm("nop");
	lcd1100_SCLK_off;
	//asm("nop");
        if(cd)
	{
		lcd1100_SDA_on;
	}
	else
	{
		lcd1100_SDA_off;
	}
        //asm("nop");
	lcd1100_SCLK_on;
        //asm("nop");
	for(i=0; i<8; i++)
	{
		lcd1100_SCLK_off;
		if(c & 0x80)
		{
			lcd1100_SDA_on;
		}
		else
		{
			lcd1100_SDA_off;
		}
		//asm("nop");
		c <<= 1;
                lcd1100_SCLK_on;
	}
        //asm("nop");
	lcd1100_CS_on;
        //asm("nop");
}

void lcd_digit48(unsigned char x, unsigned char y, unsigned digit)
{
	lcd1100_gotoxy(x, y);
	unsigned char row, col;

	for (row=0; row < FONT48_HEIGHT/8; row++)
	{
		for (col=0; col < FONT48_WIDTH; col++)
			lcd1100_write(lcd1100_DATA, digit48 [digit][row][col]);

		y += 1;
		lcd1100_gotoxy(x, y);
	}
}


//clears the 48 size digit rectangle 
void lcd_null48(unsigned char x, unsigned char y) 
{
	lcd1100_gotoxy(x, y);
	unsigned char row, col;

	for (row=0; row < FONT48_HEIGHT/8; row++)
	{
		for (col=0; col < FONT48_WIDTH; col++)
			lcd1100_write(lcd1100_DATA, 0x00);

		y += 1;
		lcd1100_gotoxy(x, y);
	}
}

void lcd_digit16(unsigned char x, unsigned char y, unsigned digit)
{
	lcd1100_gotoxy(x, y);
	unsigned char row, col;

	for (row=0; row < FONT16_HEIGHT/8; row++)
	{
		for (col=0; col < FONT16_WIDTH; col++)
			lcd1100_write(lcd1100_DATA, digit16 [digit][row][col]);

		y += 1;
		lcd1100_gotoxy(x, y);
	}
}

//	Очистка дисплея
void lcd1100_clear(void)
{
	unsigned int i;
        /*
	lcd1100_write(lcd1100_CMD,0x40);
	lcd1100_write(lcd1100_CMD,0xb0);
	lcd1100_write(lcd1100_CMD,0x10);
	lcd1100_write(lcd1100_CMD,0x00);
	lcd1100_write(lcd1100_CMD,0xae);
        lcd1100_write(lcd1100_CMD,0xaf);*/
        
        lcd1100_gotoxy(0,0);
        
	for(i=0;i<864;i++)
		lcd1100_write(lcd1100_DATA,0x00);
	
}

//	Очистка дисплея
void lcd1100_fill(uint8_t fillbyte)
{
	unsigned int i;      
        lcd1100_gotoxy(0,0);
        
	for(i=0;i<864;i++)
          lcd1100_write(lcd1100_DATA, fillbyte);	
}

//	Очистка дисплея
void lcd1100_fill_image(uint8_t * img)
{
	unsigned int i;      
        lcd1100_gotoxy(0,0);
        
	for(i=0;i<864;i++)
          lcd1100_write(lcd1100_DATA, *img++);	
}

void lcd_contrast(uint8_t value)
{
	lcd1100_write(lcd1100_CMD, 0x20 | 0x04);
	lcd1100_write(lcd1100_CMD, 0x80 |(value & 0x1F));
}

//	Инициализация дисплея
void lcd1100_init(void)
{       
	delay_lcd1100(100);
	//lcd1100_CS_off;
        delay_lcd1100(100);
	//lcd1100_RST_off;
	delay_lcd1100(100);
	//lcd1100_RST_on;
	delay_lcd1100(100);
	 
        //NOKIA 3510 BW LCD PCF8814
        lcd1100_write(lcd1100_CMD,0xE2); // *** SOFTWARE RESET
	lcd1100_write(lcd1100_CMD,0x3A); // *** Use internal oscillator
	lcd1100_write(lcd1100_CMD,0xEF); // *** FRAME FREQUENCY:
	lcd1100_write(lcd1100_CMD,0x04); // *** 80Hz
	lcd1100_write(lcd1100_CMD,0xD0); // *** 1:65 divider
	lcd1100_write(lcd1100_CMD,0xEB); // Включить температурную компенсацию
	lcd1100_write(lcd1100_CMD,0x24); // Запись в регистр Vop
	lcd1100_write(lcd1100_CMD,0x85); // Определяет контрастность
	lcd1100_write(lcd1100_CMD,0xA4); // all on/normal display
	lcd1100_write(lcd1100_CMD,0x2F); // Power control set(charge pump on/off)
	lcd1100_write(lcd1100_CMD,0x40); // set start row address = 0
	lcd1100_write(lcd1100_CMD,0xB0); // установить Y-адрес = 0
	lcd1100_write(lcd1100_CMD,0x10); // установить X-адрес, старшие 3 бита
	lcd1100_write(lcd1100_CMD,0x00);  // установить X-адрес, младшие 4 бита

        lcd1100_write(lcd1100_CMD,0x3D); //set mult factor
        lcd1100_write(lcd1100_CMD,0x03);
        
	//lcd1100_write(lcd1100_CMD,0xC8); // mirror Y axis (about X axis)
	//lcd1100_write(lcd1100_CMD,0xA1); // Инвертировать экран по горизонтали

	lcd1100_write(lcd1100_CMD,0xAC); // set initial row (R0) of the display
	lcd1100_write(lcd1100_CMD,0x07);
	lcd1100_write(lcd1100_CMD,0xAF); // экран вкл/выкл
        
 

}

//	Установка курсора
// 	Input : x,y - координаты символа
void lcd1100_gotoxy(unsigned char x,unsigned char y)
{
	lcd1100_X=x;
	lcd1100_Y=y;
	x=x*6;
	lcd1100_write(lcd1100_CMD,(0xB0|(y&0x0F)));	
	lcd1100_write(lcd1100_CMD,(0x00|(x&0x0F)));
	lcd1100_write(lcd1100_CMD,(0x10|((x>>4)&0x07)));
}

//	Преобразование символов через таблицу символов для вывода на дисплей
//	Input : с - символ в ASCII кодировке
unsigned char lcd1100_symbol_decode(unsigned char c)
{
	if(32<=c&&c<='~')
	{
		c=c-32;
	}
	else
	{
		if((192<=c)&&(c<=255))
		{
			c=c-97;
		}
		else
		{
			c=255;
		}
	}
	return c;
}

//	Отправка символа на дисплей
//	Input : с - символ в ASCII кодировке
void lcd1100_putch(unsigned char c)
{
	c=lcd1100_symbol_decode(c);
	if(c==255)
	{
		return;
	}
	lcd1100_write(lcd1100_DATA,lcd1100_font[c][0]);
	lcd1100_write(lcd1100_DATA,lcd1100_font[c][1]);
	lcd1100_write(lcd1100_DATA,lcd1100_font[c][2]);
	lcd1100_write(lcd1100_DATA,lcd1100_font[c][3]);
	lcd1100_write(lcd1100_DATA,lcd1100_font[c][4]);
	lcd1100_write(lcd1100_DATA,0x00);
	lcd1100_X++;
	if(lcd1100_X==16)
	{
		if(lcd1100_Y==7)
		{
			lcd1100_gotoxy(0,0);
		}
		else
		{
			lcd1100_gotoxy(0,lcd1100_Y+1);
		}
	}
}

void lcd1100_putch_inv(unsigned char c)
{
	c=lcd1100_symbol_decode(c);
	if(c==255)
	{
		return;
	}
	lcd1100_write(lcd1100_DATA,255-lcd1100_font[c][0]);
	lcd1100_write(lcd1100_DATA,255-lcd1100_font[c][1]);
	lcd1100_write(lcd1100_DATA,255-lcd1100_font[c][2]);
	lcd1100_write(lcd1100_DATA,255-lcd1100_font[c][3]);
	lcd1100_write(lcd1100_DATA,255-lcd1100_font[c][4]);
	lcd1100_write(lcd1100_DATA,255);
	lcd1100_X++;
	if(lcd1100_X==16)
	{
		if(lcd1100_Y==7)
		{
			lcd1100_gotoxy(0,0);
		}
		else
		{
			lcd1100_gotoxy(0,lcd1100_Y+1);
		}
	}
}

//	Отправка строки на дисплей
//	Input : s - строка
void lcd1100_puts(char *s)
{
	while (*s) 
	{
		lcd1100_putch(*s);
		++s;
	}
}

void lcd1100_puts_inv(unsigned char *s)
{
	while (*s) 
	{
		lcd1100_putch_inv(*s);
		++s;
	}
}

//	Вывод символа на дисплей х2,4,8 размера
//	Input : col - ширина, row - высота, c - символ в ASCII
//	значения ширины и высоты должны быть кратны 2: 1,2,4,8
void lcd1100_putch_big(unsigned char col,unsigned char row, char c)
{
	unsigned char i,j,k,l,variable1,shift1,variable2,shift2,shift3,t_X,t_Y;
	unsigned int sym,sym2;
	t_X=lcd1100_X;
	t_Y=lcd1100_Y;
	c=lcd1100_symbol_decode(c);
	if(c==255)
	{
		return;
	}
	lcd1100_gotoxy(lcd1100_X,lcd1100_Y);
	variable1=0;
	shift1=8/row;
	variable2=1;
	switch(row)
	{
		case 1: {shift2=1; shift3=1; }break;
		case 2: {shift2=16;shift3=2; }break;
		case 4: {shift2=4; shift3=8; }break;
		case 8: {shift2=2; shift3=32;}break;
	}
	for(i=0;i<row;i++)
	{
		for(j=0;j<5;j++)
		{
			sym=0;
			l=1;
			for(k=variable1;k<variable1+shift1;k++)
			{
				sym+=(lcd1100_font[c][j]&(1<<k))*l;
				l*=shift3;
			}
			sym=sym/variable2;

			sym2=0;
			for(k=0;k<row;k++)
			{
				sym2=sym2+sym;
				sym*=2;
			}
			for(k=0;k<col;k++)
			{
				lcd1100_write(lcd1100_DATA,sym2);
			}
		}
		lcd1100_gotoxy(lcd1100_X,lcd1100_Y+1);
		variable1+=shift1;
		variable2*=shift2;
	}
	lcd1100_X=t_X+col;
	lcd1100_Y=t_Y;
}

//	Вывод строки на дисплей х2,4,8 размера
//	Input : col - ширина, row - высота, *s - ссыла на строку
//	значения ширины и высоты должны быть кратны 2: 1,2,4,8
void lcd1100_puts_big(unsigned char col,unsigned char row, char *s)
{
	while (*s) 
	{
		lcd1100_putch_big(col,row,*s);
		++s;
	}

}

//	Тестовое заполнение дисплея подряд идущим символами
void lcd1100_test(void)
{
	unsigned char i;    
  lcd1100_gotoxy(0,0);
	for(i=32;i<127;i++) 
  {
		lcd1100_putch(i);
	}
	for(i=192;i<=224;i++) 
  {
		lcd1100_putch(i);
	}
}
