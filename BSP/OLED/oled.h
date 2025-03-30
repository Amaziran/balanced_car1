#ifndef __OLED_H__
#define __OLED_H__

#include "stm32f1xx_hal.h"
//使用硬件I2C1
///
#define   OLED_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()

#define   GPIOx_OLED_PORT               GPIOB
#define   OLED_SCK_PIN                  GPIO_PIN_8
#define   OLED_SCK_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_SET)
#define   OLED_SCK_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SCK_PIN, GPIO_PIN_RESET)
#define   OLED_SCK_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SCK_PIN)
#define   OLED_SDA_PIN                  GPIO_PIN_9
#define   OLED_SDA_ON()                 HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_SET)
#define   OLED_SDA_OFF()                HAL_GPIO_WritePin(GPIOx_OLED_PORT, OLED_SDA_PIN, GPIO_PIN_RESET)
#define   OLED_SDA_TOGGLE()             HAL_GPIO_TogglePin(GPIOx_OLED_PORT, OLED_SDA_PIN)
#define OLED_COLUMN_NUMBER 128
#define OLED_LINE_NUMBER 64
#define OLED_COLUMN_OFFSET 2
#define OLED_PAGE_NUMBER OLED_LINE_NUMBER/8
///

void WriteCmd(void);
void OLED_WR_CMD(uint8_t cmd);
void OLED_WR_DATA(uint8_t data);
void OLED_Init(void);								//oled初始化
void OLED_Clear(void);							//清屏
void OLED_Display_On(void);
void OLED_Display_Off(void);
void Page_set(unsigned char page);
void Column_set(unsigned char column);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_On(void);
void OLED_ShowNum(uint8_t x,uint8_t y,unsigned int num,uint8_t len,uint8_t size2);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t Char_Size);					//显示字符串,x为横坐标，y为纵坐标，Size有12和16两种，一般选用12
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);														//显示中文


#endif


