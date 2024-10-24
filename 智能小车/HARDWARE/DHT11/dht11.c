#include "include.h"

static GPIO_InitTypeDef GPIO_InitStruct;

void dht11_config(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;	
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT; 
	GPIO_InitStruct.GPIO_OType  = GPIO_OType_OD;  
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;   //�ⲿ������������
	GPIO_Init(GPIOG,&GPIO_InitStruct);		
	
	PGout(9) = 1;
}

void Output_mode(void)
{
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;	
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT; 
	GPIO_InitStruct.GPIO_OType =  GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;   //�ⲿ������������
	GPIO_Init(GPIOG,&GPIO_InitStruct);	
	PGout(9) = 1;	
}

void Input_mode(void)
{
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;	
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;  
	GPIO_InitStruct.GPIO_OType =  GPIO_OType_OD;	
	GPIO_InitStruct.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;   //�ⲿ������������
	GPIO_Init(GPIOG,&GPIO_InitStruct);	
}


int8_t dht11_read(uint8_t *buff)  //��ʼ�ź�     //0��û��⵽dht11   1����⵽dht11
{
	uint32_t t=0;
	int8_t i,j;
	uint8_t d=0;
	uint8_t check_sum=0;
	
	Output_mode();
	PGout(9) = 0;
	delay_ms(20);  //�������������ͱ������18����,��֤DHT11�ܼ�⵽��ʼ�źš�
	
	PGout(9) = 1;  
	delay_us(30);  //�������Ϳ�ʼ�źŽ�����,��ʱ�ȴ�20-40us��
	
	Input_mode();  //��ʼ���DHT11����Ӧ�źţ����Ը�Ϊ����ģʽ
	while(PGin(9) == 1)   //1ms������û�м�⵽���߱�dht11����(����������û���յ�dht11���ص���Ӧ�ź�)���˳�ѭ������
	{
		delay_us(1);
		t++;
		if(t>4000)	return -1;
	}
	t=0;
	
	while(PGin(9) == 0)   //dht11����80us�͵�ƽ��Ӧ�ź�,��������������ص���Ӧ�źŴ���1ms��dht11����
	{
		delay_us(1);
		t++;
		if(t>=100)	return -2;		
	}
	t=0;
	
	while(PGin(9) == 1)   //����80us�͵�ƽ�źźţ�dht11�ַ���80us�ߵ�ƽ�źţ������صĸߵ�ƽ����1ms���˳����������dht11�ɹ�������Ӧ�ź�
	{
		delay_us(1);
		t++;
		if(t>100)	return -3;			
	}
	for(j=0;j<5;j++) //ѭ������5���ֽ�
	{
		d=0;
		for(i=7;i>=0;i--) //��������һ���ֽڽ���
		{
			t=0;
			while(PGin(9) == 0)   
			{
				delay_us(1);
				t++;
				if(t>100)	return -4;			
			}
			delay_us(40);  //28-70us֮��
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
	delay_us(50);	//��ʱ50us�����Ժ���ͨѶ�����ĵ͵�ƽ
	
	check_sum=(buff[0]+buff[1]+buff[2]+buff[3])&0xff;  	//����У���

	if(check_sum == buff[4]) return 0;
	return -6;
}


