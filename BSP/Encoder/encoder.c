#include "encoder.h"


int ReadSpeed(TIM_HandleTypeDef *htim)
{
		int temp;
		temp = __HAL_TIM_GetCounter(htim);							//获取count值
		__HAL_TIM_SetCounter(htim,0);										//读取完之后将counter置0

		if(temp > 32766)
			temp = - (65536 - temp);
		return temp;																		//temp范围 (-32768,32768)
}










