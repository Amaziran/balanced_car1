#include "hc_sr04.h"


double distance = 0;
int upEdge = 0;
int downEdge = 0;
bool distance_warnning = 0;				//��ȫ�����жϱ�־(0Ϊ��ȫ��1ΪΣ��)

extern TIM_HandleTypeDef htim3;

//
void GetDistence(void)
{
	//����ģ�鷢�����壺
		HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
		//HAL_Delay(1);
		RCCdelay_us(12);
		HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
		
		//Ϊ��ֹ��ʼ����ʱ��������ֵ�Ѿ��ϴ��ڻ���½���ʱ�պ������Զ���װ�أ��õ�һ��Сֵ�������������������̽�����������
		__HAL_TIM_SET_COUNTER(&htim3,0);

}


//���벶����жϻص�����   tim3 channel3��channel4
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{
		upEdge = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
		downEdge = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
		if(upEdge < downEdge)
		{
			distance = ((downEdge - upEdge) * 0.034 )/2;									//340m/s    340 00 cm/1 000 000 us     �⵽��ʱ�����۷���ʱ�䣬����Ҫ����2
			if(distance <= 50)
				distance = distance;
			else
				distance = 100;
			//Σ�վ����ж�
			if(distance <= SaftyDistance)
				distance_warnning = 1;
			else
				distance_warnning = 0;
		}
	}
}


//us����
void RCCdelay_us(uint32_t udelay)
{
  __IO uint32_t Delay = udelay * 72 / 8;//(SystemCoreClock / 8U / 1000000U)
    //��stm32f1xx_hal_rcc.c -- static void RCC_Delay(uint32_t mdelay)
  do
  {
    __NOP();
  }
  while (Delay --);
}

