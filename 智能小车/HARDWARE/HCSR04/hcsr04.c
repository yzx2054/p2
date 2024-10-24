#include "include.h"
 

static GPIO_InitTypeDef  GPIO_InitStructure;

void hcsr04_config(void)
{
    RCC_AHB1PeriphClockCmd(HCSR_TRIQ_PORT_RCC, ENABLE);  
   // RCC_AHB1PeriphClockCmd(HCSR_ECHO_PORT_RCC, ENABLE);  

	GPIO_InitStructure.GPIO_Pin = HCSR_TRIQ_PIN;     		//触发信号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    		//输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  		//推挽模式，增加驱动电流
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //不使能上下拉电阻
	GPIO_Init(HCSR_TRIQ_PORT, &GPIO_InitStructure);	        //初始化触发信号引脚对应GPIO

	GPIO_InitStructure.GPIO_Pin = HCSR_ECHO_PIN;            //回响信号引脚  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//输入模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽输出(在输入模式下该设置无效)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//不使能上下拉电阻
	GPIO_Init(HCSR_ECHO_PORT, &GPIO_InitStructure);			//初始化回响信号引脚对应GPIO

	TRIQ_W=0;   //初始化默认低电平
	printf("hcsr04_config ok\r\n");
}

int32_t hcsr04_get_distance(void)   //
{
	uint32_t t=0;
	
	TRIQ_W = 1; //触发信号引脚
	delay_us(15);
	TRIQ_W = 0;  //触发信号引脚
	
	while(ECHO_R==0)  //回响信号引脚
	{
		t++;
		delay_us(1);
		if(t>1000000) return -1;
	}
	t=0;
	while(ECHO_R==1)
	{
		t++;
		delay_us(9);  //每9us就3mm;
		if(t>1000000) return -2;
	}
	
	t=t/2;   //来回
	return t*3;  //每t就为3mm；
}

