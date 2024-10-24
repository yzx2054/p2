#include "led.h" 

void LED_Init(void)
{    	 
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	//ʹ��GPIOEʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	//ʹ��GPIOFʱ��

	//GPIOE13,GPIOE14��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;//13�ź�14������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);					//��ʼ��GPIO
	
	GPIO_SetBits(GPIOE,GPIO_Pin_13 | GPIO_Pin_14);			
	
	//GPIOF9,GPIOF10��ʼ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;	//9�ź�10������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//����
	GPIO_Init(GPIOF, &GPIO_InitStructure);					//��ʼ��GPIO
	
	GPIO_SetBits(GPIOF,GPIO_Pin_9 | GPIO_Pin_10);	
}






