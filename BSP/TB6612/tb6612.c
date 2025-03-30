#include "tb6612.h"

//�����װ�أ�7200 - 1
#define PWM_MAX 7200
#define PWM_MIN -7200
extern TIM_HandleTypeDef htim2;


//��pid����ó���ֵ������������PWM���ʱ��Ҫ��תΪ��
int abs(int p)
{
	if(p>0)
		return p;
	else
		return -p;
}



void TB6612_Init(void)
{
	//���������IN�˶���0
	//AIN1��AIN2
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
	
	//BIN1��BIN2
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);
}


//�����Լ��ٶȿ���
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


//�޷�
void Limit(int *motoA,int *motoB)
{
	//���ڲ���ֵ���������ֵ
	if(*motoA > PWM_MAX)
		*motoA=PWM_MAX;
	
	if(*motoB > PWM_MAX)
		*motoB=PWM_MAX;
	
	//���ڲ���ֵ��С����Сֵ
	if(*motoA < PWM_MIN)
		*motoA=PWM_MIN;
	
	if(*motoB < PWM_MIN)
		*motoB=PWM_MIN;
}




//���Գ���


//������ת
void TB6612_Forward_Left(void)
{
	//IN1 == 0,IN2 == 1,��ת       IN1 == 1,IN2 == 0, ��ת         A B�෴
	//AIN1��AIN2
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET);

}

//���ַ�ת
void TB6612_Backward_Left(void)
{
	//IN1 == 0,IN2 == 1,��ת       IN1 == 1,IN2 == 0, ��ת         A B�෴
	//AIN1��AIN2
	HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET);
}


//������ת
void TB6612_Forward_Right(void)
{
	//IN1 == 1,IN2 == 0,��ת       IN1 == 0,IN2 == 1, ��ת         A B�෴
	//BIN1��BIN2
	HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET);

}

//���ַ�ת
void TB6612_Backward_Right(void)
{
	//IN1 == 1,IN2 == 0,��ת       IN1 == 0,IN2 == 1, ��ת         A B�෴
	//BIN1��BIN2
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







