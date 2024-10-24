#include "include.h"

static GPIO_InitTypeDef  GPIO_InitStructure;
static TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
static TIM_OCInitTypeDef  TIM_OCInitStructure;

void l298n_wheel_config(uint32_t duty)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);    //ʹ������IN1��ӦGPIOʱ�� 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);   //ʹ������IN2�Լ��ҵ��IN1��IN2��ӦGPIOʱ�� 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);   //ʹ�����ҵ���ٶ����Ŷ�ӦGPIOʱ�� 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);     //ʹ�ܶ�ʱ��3��Ӳ��ʱ��

/*********************************IN1-IN2-IN3-IN4**************************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;                              //��ߵ��IN1����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         					 //���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        					 //����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    					 //����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;      					 //��ʹ������������
	GPIO_Init(GPIOF, &GPIO_InitStructure);	                                 //��ʼ����ߵ��IN1-IN2��ӦGPIO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_14 | GPIO_Pin_0;     //��ߵ��IN2����,�ұߵ��IN1-IN2����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         					 //���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                           //����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                       //����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                         //��ʹ������������
	GPIO_Init(GPIOD, &GPIO_InitStructure);	                                 //��ʼ����ߵ��IN2���ұߵ��IN1��IN2��ӦGPIO

	LEFT_IN1=1;                                                              //��ʼ��Ϊɲ��״̬(IN1,IN2,IN3,IN4ȫΪ0��1);
	LEFT_IN2=1;
	RIGHT_IN1=1;
	RIGHT_IN2=1;
/**********************************���ҵ���ٶ�**************************************************************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9;          //���ҵ���ٶ�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;         	        //����ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  //����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              //����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                //��ʹ������������
	GPIO_Init(GPIOC, &GPIO_InitStructure);                          //��ʼ�����ҵ���ٶ����Ŷ�ӦGPIO
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);         //��PC7��������Ϊ��ʱ��3��PWM����
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);         //��PC9��������Ϊ��ʱ��3��PWM����

	TIM_TimeBaseStructure.TIM_Period = (10000/100)-1;			    //���ö�ʱ��1���������Ƶ��100Hz,��Ӧ����ֵ(10000/100-1)
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;				    //Ԥ��Ƶֵ��Ҳ�������Ϊ��һ�η�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	    //���ϼ���
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);                 

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//������PWM1ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�������
	TIM_OCInitStructure.TIM_Pulse = duty;								//�Ƚ�ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//��Ч״̬���ߵ�ƽ�������Ч״̬���͵�ƽ���
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//������PWM1ģʽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//�������
	TIM_OCInitStructure.TIM_Pulse = duty;								//�Ƚ�ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//��Ч״̬���ߵ�ƽ�������Ч״̬���͵�ƽ���
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_Cmd(TIM3, ENABLE);	  	                                    //������ʱ��3

	printf("l298n_wheel_config ok\r\n");

}



