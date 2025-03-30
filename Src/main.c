/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//OLED:
//	SCL:PB8
//  SDA:PB9

//HC-SR04  超声波测距：
//  Trig:PA11    (GPIO_output)
//	Echo:PB0		 (TIM3输入捕获)


//tb6612
//	PWMA:PA0
//	AIN1:PC14
//	AIN2:PC15
//	STBY:3.3V
//	VM:12V/11.47V
//	VCC:3.3V
//	AO1:M1
//	AO2:M2
//	PWMB:PA3
//	BIN1:PA5
//	BIN2:PA6
//	BO1:M1
//	BO2:M2

//左520电机
//	VCC:3.3V
//	C1:PA9			TIM1 channel2			B相
//	C2:PA8			TIM1 channel1			A相
	
//右电机
//	C1:PB7			TIM4 channel2			B相
//	C2:PB6			TIM4 channel1			A相

//MPU6050
//	SCL:PB4
//	SDA:PB3
//	INT:PB5			EXIT5,触发control

//HC-06蓝牙
//	VCC:3.3
//	单片机TX:PB10  <-->  蓝牙RX
//	单片机RX:PB11  <-->  蓝牙TX
//	获取APP指令的中断在stm32f1xx_it.c文件234行


#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "oled.h"
#include "tb6612.h"
#include "hc_sr04.h"
#include "IIC.h"
#include "inv_mpu.h"
#include "mpu6050.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "encoder.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
char message[20] = "";

int count = 0;

extern float pitch,roll,yaw;								//陀螺仪三相参数

uint32_t sys_tick = 0;

extern int Encoder_Left,Encoder_Right;

extern double distance;

extern uint8_t rx_buf[2];

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void read(void); 

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	
	OLED_Init();
	TB6612_Init();
	MPU_Init();																				//陀螺仪初始化
	mpu_dmp_init();																		//dmp初始化
	Load(0,0);																				//电机初始速度为0
	HAL_TIM_Base_Start(&htim3);												//tim3无更新中断，不加_IT,捕获时钟(hc-sr04)
	//HAL_TIM_Base_Start_IT(&htim3);									//tim3的内部时钟中断，用于测量陀螺仪角度
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);					//设置外部中断优先级
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);									//使能外部中断
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);					//左电机
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);					//右电机
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);		//编码器
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);		//编码器
	HAL_TIM_IC_Start(&htim3,TIM_CHANNEL_3);						//启动输入捕获的语句，IC：Input Capture
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_4);				//在获得两个捕获的值之后需要进行计算，所以需要开一个比较中断，加个_IT
	HAL_UART_Receive_IT(&huart3,rx_buf,1);						
	
	
	OLED_Clear();
	sprintf(message,"Init is OK");
	OLED_ShowString(1,3,(uint8_t *)message,12);
	HAL_Delay(500);
	OLED_Clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			
//			sprintf(message,"Encoder_L:%d   ",Encoder_Left);
//			OLED_ShowString(1,0,(uint8_t *)message,12);
//			sprintf(message,"Encoder_R:%d   ",Encoder_Right);
//			OLED_ShowString(1,1,(uint8_t *)message,12);
//			sprintf(message,"roll:%.1f   ",roll);
//			OLED_ShowString(1,2,(uint8_t *)message,12);
//			GetDistence();
//			sprintf(message,"dis:%.1fcm  ",distance);
//			OLED_ShowString(1,3,(uint8_t *)message,12);
		
			//一定要尝试加在陀螺仪给的中断里面
			Control();
	
		
			
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

//内部时钟中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef  *htim)
{
	if(htim == &htim3)
	{
		
	}
}


//
void read(void)
{
	//在系统定义中，uwTick每1ms递增一次
	if(uwTick - sys_tick < 10)							//10ms触发一次
		return ;
	sys_tick = uwTick;
	Encoder_Left = ReadSpeed(&htim1);
	
	Encoder_Right = - ReadSpeed(&htim4);
}
	
	


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
