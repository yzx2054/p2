#include "include.h"

static GPIO_InitTypeDef  GPIO_InitStructure;
static TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
static TIM_OCInitTypeDef TIM_OCInitStruct;

/*************************PWMģʽ���*********************************/
void sg90_config(void)  
{
	//ʹ�ܶ˿�Dʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	/* TIM2 clock enable ����ʱ��4��ʱ��ʹ��*/	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//����PA2�����ù��ܣ�
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15 ; 	//15 �����Ŷ�ӦTIM4CH4
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;    //����Ϊ���ù���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//����ģʽ��������������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//����IO���ٶ�Ϊ100MHz��Ƶ��Խ������Խ�ã�Ƶ��Խ�ͣ�����Խ��
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;//��ʹ������������
	GPIO_Init(GPIOD,&GPIO_InitStructure);	

	//��������֧�ֺܶ๦�ܣ���Ҫָ�������ŵĹ��ܣ���ǰҪ�ƶ�֧��TIM4
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);   //��סҪ��������
	
	/* Time base configuration����ʱ���Ļ������ã��������ö�ʱ������������Ƶ��Ϊ50Hz */
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1; //�ڶ��η�Ƶ,��ǰʵ��1��Ƶ��Ҳ���ǲ���Ƶ
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=10000/50;	 //���ö�ʱ��Ƶ��Ϊ50Hz
	TIM_TimeBaseInitStruct.TIM_Prescaler=8400-1;  //��һ�η�Ƶ�����ΪԤ��Ƶ
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);

	/* PWM1 Mode configuration: Channel4 ����PWM��ͨ��4������ģʽ1*/
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;              //PWMģʽ1���ڵ���ģʽ�£�ֻҪ TIMx_CNT < TIMx_CCR1��ͨ�� 4 ��Ϊ��Ч״̬���ߵ�ƽ��������Ϊ��Ч״̬���͵�ƽ����
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;      //��Ч��ʱ������ߵ�ƽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  //�������
	//TIM_OCInitStruct.TIM_Pulse = 0;   
	TIM_OC4Init(TIM4,&TIM_OCInitStruct);	
	
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);	//��ʱ��ͨ��4�Զ����س�ֵ���������PWM����

	TIM_ARRPreloadConfig(TIM4, ENABLE);			//�Զ����س�ֵʹ��
	
	/* TIM4 enable counter��ʹ�ܶ�ʱ��4���� */
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
