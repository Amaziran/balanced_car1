#ifndef __PID_H__
#define __PID_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_it.h"

void Control(void);	//每隔10ms调用一次

#endif
