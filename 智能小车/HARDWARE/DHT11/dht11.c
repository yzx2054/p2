#include "include.h"

static GPIO_InitTypeDef GPIO_InitStruct;

void dht11_config(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;	
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT; 
	GPIO_InitStruct.GPIO_OType  = GPIO_OType_OD;  
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;   //外部有上拉电阻了
	GPIO_Init(GPIOG,&GPIO_InitStruct);		
	
	PGout(9) = 1;
}

void Output_mode(void)
{
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;	
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT; 
	GPIO_InitStruct.GPIO_OType =  GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;   //外部有上拉电阻了
	GPIO_Init(GPIOG,&GPIO_InitStruct);	
	PGout(9) = 1;	
}

void Input_mode(void)
{
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;	
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;  
	GPIO_InitStruct.GPIO_OType =  GPIO_OType_OD;	
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;   //外部有上拉电阻了
	GPIO_Init(GPIOG,&GPIO_InitStruct);	
}


int8_t dht11_read(uint8_t *buff)  //起始信号     //0：没检测到dht11   1：检测到dht11
{
	uint32_t t=0;
	int8_t i,j;
	uint8_t d=0;
	uint8_t check_sum=0;
	
	Output_mode();
	PGout(9) = 0;
	delay_ms(20);  //主机把总线拉低必须大于18毫秒,保证DHT11能检测到起始信号。
	
	PGout(9) = 1;  
	delay_us(30);  //主机发送开始信号结束后,延时等待20-40us后
	
	Input_mode();  //开始检测DHT11的响应信号，所以改为输入模式
	while(PGin(9) == 1)   //1ms后若还没有检测到总线被dht11拉低(即代表主机没有收到dht11返回的响应信号)则退出循环结束
	{
		delay_us(1);
		t++;
		if(t>4000)	return -1;
	}
	t=0;
	
	while(PGin(9) == 0)   //dht11返回80us低电平响应信号,这里设计了若返回的响应信号大于1ms则dht11错误
	{
		delay_us(1);
		t++;
		if(t>=100)	return -2;		
	}
	t=0;
	
	while(PGin(9) == 1)   //返回80us低电平信号号，dht11又返回80us高电平信号，若返回的高电平大于1ms则退出，否则代表dht11成功返回响应信号
	{
		delay_us(1);
		t++;
		if(t>100)	return -3;			
	}
	for(j=0;j<5;j++) //循环接收5个字节
	{
		d=0;
		for(i=7;i>=0;i--) //进行正常一个字节接收
		{
			t=0;
			while(PGin(9) == 0)   
			{
				delay_us(1);
				t++;
				if(t>100)	return -4;			
			}
			delay_us(40);  //28-70us之间
			if(PGin(9))
			{
				d|=1<<i;
				t=0;
				while(PGin(9) == 1)   
				{
					delay_us(1);
					t++;
					if(t>100)	return -5;			
				}
			}
		}
		buff[j]=d;
	}
	delay_us(50);	//延时50us，可以忽略通讯结束的低电平
	
	check_sum=(buff[0]+buff[1]+buff[2]+buff[3])&0xff;  	//进行校验和

	if(check_sum == buff[4]) return 0;
	return -6;
}


