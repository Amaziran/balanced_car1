#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"



void TB6612_Init(void);
void TB6612_Forward_Left(void);
void TB6612_Backward_Left(void);
void TB6612_Forward_Right(void);
void TB6612_Backward_Right(void);
void TB6612_PWMA(int count);
void TB6612_PWMB(int count);

void Load(int moto1,int moto2);
void Limit(int *motoA,int *motoB);

