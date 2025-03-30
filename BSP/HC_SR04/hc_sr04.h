#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "oled.h"
#include "stdbool.h"

//°²È«¾àÀë
#define  SaftyDistance  10



void GetDistence(void);


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

void RCCdelay_us(uint32_t udelay);


