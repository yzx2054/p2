#include "include.h"

static GPIO_InitTypeDef  GPIO_InitStructure;

void beep_config(void)     //蜂鸣器初始化
{
    RCC_AHB1PeriphClockCmd(BEEP_PORT_RCC, ENABLE);    //使能蜂鸣器对应GPIO时钟 

	GPIO_InitStructure.GPIO_Pin = BEEP_PIN;             //蜂鸣器引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         //输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //推挽模式，增加驱动电流
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      //不使能上下拉电阻
	GPIO_Init(BEEP_PORT, &GPIO_InitStructure);	              //初始化GPIO
	
	GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);     //对应引脚设置低电平，蜂鸣器不响

	printf("beep_config ok\r\n");
}





