#include "encoder.h"


int ReadSpeed(TIM_HandleTypeDef *htim)
{
		int temp;
		temp = __HAL_TIM_GetCounter(htim);							//��ȡcountֵ
		__HAL_TIM_SetCounter(htim,0);										//��ȡ��֮��counter��0

		if(temp > 32766)
			temp = - (65536 - temp);
		return temp;																		//temp��Χ (-32768,32768)
}










