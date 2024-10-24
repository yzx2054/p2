#include "include.h"
 

static GPIO_InitTypeDef  GPIO_InitStructure;

void hcsr04_config(void)
{
    RCC_AHB1PeriphClockCmd(HCSR_TRIQ_PORT_RCC, ENABLE);  
   // RCC_AHB1PeriphClockCmd(HCSR_ECHO_PORT_RCC, ENABLE);  

	GPIO_InitStructure.GPIO_Pin = HCSR_TRIQ_PIN;     		//�����ź�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;    		//���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  		//����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //��ʹ������������
	GPIO_Init(HCSR_TRIQ_PORT, &GPIO_InitStructure);	        //��ʼ�������ź����Ŷ�ӦGPIO

	GPIO_InitStructure.GPIO_Pin = HCSR_ECHO_PIN;            //�����ź�����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //�������(������ģʽ�¸�������Ч)
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//��ʹ������������
	GPIO_Init(HCSR_ECHO_PORT, &GPIO_InitStructure);			//��ʼ�������ź����Ŷ�ӦGPIO

	TRIQ_W=0;   //��ʼ��Ĭ�ϵ͵�ƽ
	printf("hcsr04_config ok\r\n");
}

int32_t hcsr04_get_distance(void)   //
{
	uint32_t t=0;
	
	TRIQ_W = 1; //�����ź�����
	delay_us(15);
	TRIQ_W = 0;  //�����ź�����
	
	while(ECHO_R==0)  //�����ź�����
	{
		t++;
		delay_us(1);
		if(t>1000000) return -1;
	}
	t=0;
	while(ECHO_R==1)
	{
		t++;
		delay_us(9);  //ÿ9us��3mm;
		if(t>1000000) return -2;
	}
	
	t=t/2;   //����
	return t*3;  //ÿt��Ϊ3mm��
}

