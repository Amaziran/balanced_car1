#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "tb6612.h"
#include "stdbool.h"


#define SPEED_Y 30 //俯仰(前后)最大设定速度
#define SPEED_Z 150//偏航(左右)最大设定速度 


//传感器数据变量
int Encoder_Left,Encoder_Right;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short	aacx,aacy,aacz;


//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
float Med_Angle=3;//平衡时角度值偏移量（机械中值）

//参数
float Vertical_Kp=180,Vertical_Kd=2;			//直立环 数量级（Kp：0~1000、Kd：0~10）
float Velocity_Kp=0.6,Velocity_Ki=0.003;		//速度环 数量级（Kp：0~1）
float Turn_Kp=10,Turn_Kd=0.6;											//转向环

uint8_t stop;

extern TIM_HandleTypeDef htim2,htim4;
extern float distance;
extern uint8_t Fore,Back,Left,Right;										//前后左右为蓝牙APP所传输数据
extern bool distance_warnning;


//直立环PD控制器
//输入：期望角度、真实角度、角速度
int Vertical(float Med,float Angle,float gyro_Y)
{
	int temp;
	//调角度的因数
	temp = 1.25 * Vertical_Kp * (Angle - Med) + Vertical_Kd * gyro_Y;
	return temp;
}


//速度环PI控制器
//输入：期望速度、左编码器、右编码器
int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S;
	static float a=0.7;
	int Err,Err_LowOut,temp;
	Velocity_Ki = Velocity_Kp / 200;
	//1、计算偏差值
	Err = ( encoder_L + encoder_R ) - Target;
	//2、低通滤波
	Err_LowOut = (1 - a) * Err+a * Err_LowOut_last;
	Err_LowOut_last = Err_LowOut;
	//3、积分
	Encoder_S += Err_LowOut;
	//4、积分限幅(-20000~20000)
	Encoder_S = Encoder_S > 20000 ? 20000:(Encoder_S < (-20000) ? (-20000):Encoder_S);
	if(stop == 1)
		Encoder_S = 0,stop = 0;
	//5、速度环计算
	temp = Velocity_Kp * Err_LowOut + Velocity_Ki * Encoder_S;
	return temp;
}


//转向环PD控制器
//输入：角速度、角度值
int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp = Turn_Kp * Target_turn + Turn_Kd * gyro_Z;
	return temp;
}

//PID控制，每隔10ms调用一次
void Control(void)
{
	int PWM_out;
	
	//1、读取编码器和陀螺仪的数据
	Encoder_Left = ReadSpeed(&htim1);										//左
	Encoder_Right= - ReadSpeed(&htim4);									//右
	mpu_dmp_get_data(&pitch,&roll,&yaw);								//陀螺仪偏移量
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);						//角速度
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);						//加速度
	
	//蓝牙APP遥控
//	if((Fore == 0) && (Back == 0))											//未接受到前进后退指令-->速度清零，稳在原地
//		Target_Speed=0;
	
	if(Fore == 1)
	{
		Target_Speed = 10;
	}
	
	else if(Back == 1)
	{
		Target_Speed = -10;
	}
	else
	{
		Target_Speed = 0;
	}
	
	//限幅   两个三目运算
	Target_Speed = Target_Speed > SPEED_Y ? SPEED_Y:(Target_Speed < -SPEED_Y ? (-SPEED_Y):Target_Speed);
	
	/*左右*/
	if((Left==0)&&(Right==0))
		Target_turn = 0;
	if(Left==1)
		Target_turn -= 30;	//左转
	if(Right==1)
		Target_turn += 30;	//右转
	
	//限幅( (20*100) * 100   )
	Target_turn = Target_turn > SPEED_Z ? SPEED_Z :(Target_turn < -SPEED_Z ? (-SPEED_Z):Target_turn);
	
	/*转向约束*/
	if((Left==0)&&(Right==0))
		Turn_Kd=0.6;																		//若无左右转向指令，则开启转向约束
	else if((Left==1)||(Right==1))
		Turn_Kd=0;																			//若左右转向指令接收到，则去掉转向约束
	
	
	//2、将数据传入PID控制器，计算输出结果，即左右电机转速值
	//距离小于安全距离时，强制Target_Speed为 -10
	if(distance_warnning == 1)
		Velocity_out = Velocity(-10,Encoder_Left,Encoder_Right);									//仅直立，算出应给角速度
	else
		Velocity_out = Velocity(Target_Speed,Encoder_Left,Encoder_Right);					//在有期望速度的前提下直立，算出应给角速度
	
	//调整roll的角度
	Vertical_out = Vertical(Velocity_out+Med_Angle,roll+7.25,gyrox);				//速度
	Turn_out = Turn(gyroz,Target_turn);																					//偏转
	PWM_out = Vertical_out;
	MOTO1 = PWM_out - Turn_out;
	MOTO2 = PWM_out + Turn_out;
	
	//限幅
	Limit(&MOTO1,&MOTO2);
	//调速
	Load(MOTO1,MOTO2);
}



//可能是触发间隔较短，试试调长点，在MPU_Init函数中
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	//PB5接陀螺仪的INT中断
//	if(GPIO_Pin == GPIO_PIN_5)
//			Control();
//	
//}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == GPIO_PIN_5)
//	{	Control();
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
//	}
//}






