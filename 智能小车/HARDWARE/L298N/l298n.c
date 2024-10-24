#include "include.h"

static GPIO_InitTypeDef  GPIO_InitStructure;
static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
static TIM_OCInitTypeDef  TIM_OCInitStructure;

void l298n_wheel_config(uint32_t duty)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);    //使能左电机IN1对应GPIO时钟 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);   //使能左电机IN2以及右电机IN1和IN2对应GPIO时钟 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);   //使能左右电机速度引脚对应GPIO时钟 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);     //使能定时器3的硬件时钟

/*********************************IN1-IN2-IN3-IN4**************************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;                              //左边电机IN1引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         					 //输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        					 //推挽模式，增加驱动电流
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    					 //设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      					 //不使能上下拉电阻
	GPIO_Init(GPIOF, &GPIO_InitStructure);	                                 //初始化左边电机IN1-IN2对应GPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_14 | GPIO_Pin_0;     //左边电机IN2引脚,右边电机IN1-IN2引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         					 //输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                           //推挽模式，增加驱动电流
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                       //设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                         //不使能上下拉电阻
	GPIO_Init(GPIOD, &GPIO_InitStructure);	                                 //初始化左边电机IN2及右边电机IN1和IN2对应GPIO

	LEFT_IN1=1;                                                              //初始化为刹车状态(IN1,IN2,IN3,IN4全为0或1);
	LEFT_IN2=1;
	RIGHT_IN1=1;
	RIGHT_IN2=1;
/**********************************左右电机速度**************************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9;          //左右电机速度引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         	        //复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  //推挽模式，增加驱动电流
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //不使能上下拉电阻
	GPIO_Init(GPIOC, &GPIO_InitStructure);                          //初始化左右电机速度引脚对应GPIO
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);         //将PC7引脚配置为定时器3的PWM功能
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);         //将PC9引脚配置为定时器3的PWM功能

	TIM_TimeBaseStructure.TIM_Period = (10000/100)-1;			    //配置定时器1参数：输出频率100Hz,对应重载值(10000/100-1)
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;				    //预分频值，也可以理解为第一次分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	    //向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//工作在PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//输出开关
	TIM_OCInitStructure.TIM_Pulse = duty;								//比较值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//有效状态：高电平输出；无效状态：低电平输出
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//工作在PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//输出开关
	TIM_OCInitStructure.TIM_Pulse = duty;								//比较值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//有效状态：高电平输出；无效状态：低电平输出
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_Cmd(TIM3, ENABLE);	  	                                    //启动定时器3

	printf("l298n_wheel_config ok\r\n");

}



