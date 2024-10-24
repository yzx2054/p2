#include "include.h"

static GPIO_InitTypeDef  GPIO_InitStructure;

void beep_config(void)     //��������ʼ��
{
    RCC_AHB1PeriphClockCmd(BEEP_PORT_RCC, ENABLE);    //ʹ�ܷ�������ӦGPIOʱ�� 

	GPIO_InitStructure.GPIO_Pin = BEEP_PIN;             //����������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         //���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      //��ʹ������������
	GPIO_Init(BEEP_PORT, &GPIO_InitStructure);	              //��ʼ��GPIO
	
	GPIO_WriteBit(BEEP_PORT,BEEP_PIN,Bit_RESET);     //��Ӧ�������õ͵�ƽ������������

	printf("beep_config ok\r\n");
}





