#include "include.h"

void rtc_config(void)
{
	EXTI_InitTypeDef	EXTI_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
	
	RTC_TimeTypeDef  	RTC_TimeStructure;
	RTC_DateTypeDef  	RTC_DateStructure;
	RTC_InitTypeDef  	RTC_InitStructure;	
	//ʹ��rtc��Ӳ��ʱ��
	
	/* Enable the PWR clock��ʹ�ܵ�Դ����ʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	
	/* Allow access to RTC���������RTC */
	PWR_BackupAccessCmd(ENABLE);

#if 1
	//��LSE��ʱ��
	RCC_LSEConfig(RCC_LSE_ON);

	//���LSE��ʱ���Ƿ����
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
		printf("2");
	}
	
	//ΪRTCѡ��LSE��Ϊʱ��Դ
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	
#else
	
	RCC_LSICmd(ENABLE);
	
	/* Wait till LSI is ready */  
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	{
	}
	
	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
#endif
	
	/* Enable the RTC Clock��ʹ��RTC��Ӳ��ʱ�� */
	RCC_RTCCLKCmd(ENABLE);
	
	/* Wait for RTC APB registers synchronisation���ȴ�����RTC��ؼĴ������� */
	RTC_WaitForSynchro();
	
	//����Ƶ��1Hz
	/* ck_spre(1Hz) = RTCCLK(LSE) /(uwAsynchPrediv + 1)*(uwSynchPrediv + 1)*/
	//32768Hz/(127+1)/(255+1)=1Hz
	RTC_InitStructure.RTC_AsynchPrediv = 127;
	RTC_InitStructure.RTC_SynchPrediv = 255;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&RTC_InitStructure);
	
	
	//��������
	RTC_DateStructure.RTC_Year = 0x21;				//2021��
	RTC_DateStructure.RTC_Month = RTC_Month_July;	//7�·�
	RTC_DateStructure.RTC_Date = 0x21;				//��21��/21��/21��
	RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Wednesday;//������
	RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);


	//����ʱ�� 17:14:50
	RTC_TimeStructure.RTC_H12     = RTC_H12_PM;
	RTC_TimeStructure.RTC_Hours   = 0x0;
	RTC_TimeStructure.RTC_Minutes = 0x0;
	RTC_TimeStructure.RTC_Seconds = 0x0; 
	RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);  
	
	
	RTC_WakeUpCmd(DISABLE);
	
	//����ʱ��Դ��Ӳ��ʱ��Ƶ��Ϊ1Hz
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	
	//ֻ����һ�μ���
	RTC_SetWakeUpCounter(0);
	
	RTC_WakeUpCmd(ENABLE);

	//�����жϵĴ�����ʽ�������ж�
	RTC_ITConfig(RTC_IT_WUT, ENABLE);
	
	
	RTC_ClearFlag(RTC_FLAG_WUTF);
	
	EXTI_ClearITPendingBit(EXTI_Line22);
	EXTI_InitStructure.EXTI_Line = EXTI_Line22;			//
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//RTC�ֲ�涨
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//���ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	printf("rtc_config ok\r\n");

}

void rtc_alarm_set(RTC_AlarmTypeDef RTC_AlarmStructure)
{
	/* �ر����ӣ������رգ��������Ӵ������ж���BUG��������ô���ã�ֻҪ��00�룬�򴥷��ж�*/
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	
	/* Configure the RTC Alarm A register������RTC��A���� */
	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
	
	/* Enable the alarm ����RTC������A����*/
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}


void  RTC_WKUP_IRQHandler(void)
{
	uint32_t ulReturn;
	
	/* �����ٽ�Σ��ٽ�ο���Ƕ�� */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();	
	
	//����־λ
	if(RTC_GetITStatus(RTC_IT_WUT) == SET)
	{
		//�����¼���־��
		xEventGroupSetBitsFromISR(g_event_group,EVENT_GROUP_RTC_WAKEUP,NULL);	
		
		//��ձ�־λ	
		RTC_ClearITPendingBit(RTC_IT_WUT);
		
		EXTI_ClearITPendingBit(EXTI_Line22);
	}
	
	/* �˳��ٽ�� */
	taskEXIT_CRITICAL_FROM_ISR( ulReturn );			

}
