# balanced_car1
基于STM32的PID平衡小车

该文档中包括了PCB板、STM32CubeMX引脚配置文件以及所有的keil文件，PID算法原理见'PID闭环控制.png',这里说明各部分功能和引脚配置

//OLED:
//	SCL:PB8
//  SDA:PB9

//HC-SR04：
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
//	获取APP指令的中断在stm32f1xx_it.c文件234行**



