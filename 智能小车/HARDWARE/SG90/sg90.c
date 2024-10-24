#include "include.h"

static GPIO_InitTypeDef  GPIO_InitStructure;
static TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
static TIM_OCInitTypeDef TIM_OCInitStruct;

/*************************PWM模式输出*********************************/
void sg90_config(void)  
{
	//使能端口D时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	/* TIM2 clock enable ，定时器4的时钟使能*/	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//配置PA2（复用功能）
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15 ; 	//15 号引脚对应TIM4CH4
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;    //设置为复用功能模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽模式，增加驱动电流
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//不使能上下拉电阻
	GPIO_Init(GPIOD,&GPIO_InitStructure);	

	//由于引脚支持很多功能，需要指定该引脚的功能，当前要制定支持TIM4
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);   //记住要开启复用
	
	/* Time base configuration，定时器的基本配置，用于配置定时器的输出脉冲的频率为50Hz */
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1; //第二次分频,当前实现1分频，也就是不分频
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=10000/50;	 //设置定时的频率为50Hz
	TIM_TimeBaseInitStruct.TIM_Prescaler=8400-1;  //第一次分频，简称为预分频
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);

	/* PWM1 Mode configuration: Channel4 ，让PWM的通道4工作在模式1*/
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;              //PWM模式1，在递增模式下，只要 TIMx_CNT < TIMx_CCR1，通道 4 便为有效状态（高电平），否则为无效状态（低电平）。
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;      //有效的时候，输出高电平
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  //允许输出
	//TIM_OCInitStruct.TIM_Pulse = 0;   
	TIM_OC4Init(TIM4,&TIM_OCInitStruct);	
	
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	//定时器通道4自动重载初值，不断输出PWM脉冲

	TIM_ARRPreloadConfig(TIM4, ENABLE);			//自动重载初值使能
	
	/* TIM4 enable counter，使能定时器4工作 */
	TIM_Cmd(TIM4,ENABLE);
	printf("sg90_config ok\r\n");
	
}

void sg90_angle(uint32_t angle)
{
	if(angle==0)
		TIM_SetCompare4(TIM4,(uint32_t)(0.5 * 200/20));

	if(angle==45)
		TIM_SetCompare4(TIM4,(uint32_t)(200/20));
	if(angle==90)
		TIM_SetCompare4(TIM4,(uint32_t)(1.5*200/20));
	
	if(angle==135)
		TIM_SetCompare4(TIM4,(uint32_t)(2*200/20));
	
	if(angle==170)
		TIM_SetCompare4(TIM4,(uint32_t)(2.38*200/20));		
}
