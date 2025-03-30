#include "tb6612.h"

//最大重装载：7200 - 1
#define PWM_MAX 7200
#define PWM_MIN -7200
extern TIM_HandleTypeDef htim2;


//由pid计算得出的值有正负，换成PWM输出时需要都转为正
int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}



void TB6612_Init(void)
{
	//两个电机的IN端都置0
	//AIN1、AIN2
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	
	//BIN1、BIN2
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
}


//方向以及速度控制
void Load(int moto1,int moto2)			//-7200~7200
{
	if(moto1 > 0)
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);
	}
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,abs(moto1));
	
	if(moto2 > 0)
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,abs(moto2));
}


//限幅
void Limit(int *motoA,int *motoB)
{
	//调节参数值不大于最大值
	if(*motoA > PWM_MAX)
		*motoA=PWM_MAX;
	
	if(*motoB > PWM_MAX)
		*motoB=PWM_MAX;
	
	//调节参数值不小于最小值
	if(*motoA < PWM_MIN)
		*motoA=PWM_MIN;
	
	if(*motoB < PWM_MIN)
		*motoB=PWM_MIN;
}




//测试程序


//左轮正转
void TB6612_Forward_Left(void)
{
	//IN1 == 0,IN2 == 1,正转       IN1 == 1,IN2 == 0, 反转         A B相反
	//AIN1、AIN2
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);

}

//左轮反转
void TB6612_Backward_Left(void)
{
	//IN1 == 0,IN2 == 1,正转       IN1 == 1,IN2 == 0, 反转         A B相反
	//AIN1、AIN2
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
}


//右轮正转
void TB6612_Forward_Right(void)
{
	//IN1 == 1,IN2 == 0,正转       IN1 == 0,IN2 == 1, 反转         A B相反
	//BIN1、BIN2
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);

}

//右轮反转
void TB6612_Backward_Right(void)
{
	//IN1 == 1,IN2 == 0,正转       IN1 == 0,IN2 == 1, 反转         A B相反
	//BIN1、BIN2
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET);
}


//
void TB6612_PWMA(int count)
{
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,count);
}

//
void TB6612_PWMB(int count)
{
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,count);
}







