#include "hc_sr04.h"


double distance = 0;
int upEdge = 0;
int downEdge = 0;
bool distance_warnning = 0;				//安全距离判断标志(0为安全，1为危险)

extern TIM_HandleTypeDef htim3;

//
void GetDistence(void)
{
	//向测距模块发送脉冲：
		HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
		//HAL_Delay(1);
		RCCdelay_us(12);
		HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
		
		//为防止开始测试时计数器的值已经较大，在获得下降沿时刚好遇到自动重装载，得到一个小值，所以在输出脉冲后立刻将计数器清零
		__HAL_TIM_SET_COUNTER(&htim3,0);

}


//输入捕获的中断回调函数   tim3 channel3、channel4
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		upEdge = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
		downEdge = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
		if(upEdge < downEdge)
		{
			distance = ((downEdge - upEdge) * 0.034 )/2;									//340m/s    340 00 cm/1 000 000 us     测到的时间是折返的时间，所以要除以2
			if(distance <= 50)
				distance = distance;
			else
				distance = 100;
			//危险距离判断
			if(distance <= SaftyDistance)
				distance_warnning = 1;
			else
				distance_warnning = 0;
		}
	}
}


//us函数
void RCCdelay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
    //见stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}

