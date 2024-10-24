#include "include.h"

/* �����ʱ����� */
static TimerHandle_t soft_timer_Handle =NULL;  
/* �����ʱ�� */
static void soft_timer_callback(void* parameter);   //1s��һ��ι��
//��������
static StackType_t  TimerTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t TimerTaskTCB;
//��ʱ�����������ڴ�
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
	*ppxTimerTaskTCBBuffer=&TimerTaskTCB;
	*ppxTimerTaskStackBuffer=TimerTaskStack; 
	*pulTimerTaskStackSize=configMINIMAL_STACK_SIZE;
}

/* ������ */ 
static TaskHandle_t app_task_init_handle         = NULL;    //��ʼ�����   
static TaskHandle_t app_task_sg90_handle         = NULL;    //������ 
static TaskHandle_t app_task_oled_handle         = NULL;    //OLED��Ļ��� 
static TaskHandle_t app_task_hcsr04_handle		 	 = NULL;  	//���������
static TaskHandle_t app_task_beep_handle		 		 = NULL;    //���������
static TaskHandle_t app_task_bluetooth_handle		 = NULL;    //�������
static TaskHandle_t app_task_l298n_wheel_handle	 = NULL;    //L298N(����)���
static TaskHandle_t app_task_mpu6050_handle			 = NULL;    //mpu6050���
static TaskHandle_t app_task_rtc_handle					 = NULL;		//RTC���
static TaskHandle_t app_task_dht11_handle 			 = NULL;		//DHT11���
static TaskHandle_t app_task_mfrc522_handle 	 	 = NULL;		//mfrc522���

//static TaskHandle_t app_task_cpu_status_handle = NULL;


/* ������ */
static void app_task_init(void *pvParameters);    			//����  Ӳ��/�����ʼ�� 
static void app_task_sg90(void *pvParameters);    			//����  ���
static void app_task_oled(void *pvParameters);    			//����  oled��Ļ
static void app_task_hcsr04(void *pvParameters);   	 		//����  ���������
static void app_task_beep(void *pvParameters);     			//����  ������
static void app_task_bluetooth(void *pvParameters);     //����  ����
static void app_task_l298n_wheel(void *pvParameters);   //����  L298N(����)
static void app_task_mpu6050(void *pvParameters);       //����	mpu6050 
static void app_task_rtc(void* pvParameters);   				//���� 	rtc 
static void app_task_dht11(void* pvParameters);   			//����	dht11
static void app_task_mfrc522(void* pvParameters);   		//����	mfrc522

//static void app_task_cpu_status(void *pvParameters);        //����  ��ѯCPUʹ��״̬


/* ��Ϣ���о�� */
QueueHandle_t   g_queue_bluetooth;    //���� 
QueueHandle_t   g_queue_sg90;         //���
QueueHandle_t   g_queue_hcsr04;       //������
QueueHandle_t   g_queue_oled;        	//OLED��Ļ
QueueHandle_t   g_queue_beep;        	//������

/* �������ź������ */
SemaphoreHandle_t g_mutex_printf;     //USART1
SemaphoreHandle_t g_mutex_oled;

/* �������ź��� */
SemaphoreHandle_t g_sem_sg90;
SemaphoreHandle_t g_sem_beep;
SemaphoreHandle_t g_sem_hcsr04;
SemaphoreHandle_t g_sem_oled;
/* �¼���־���� */
EventGroupHandle_t g_event_group;	

/* ȫ�ֱ��� */ 
static volatile  uint32_t g_wheel_mode 	 	= MANUAL_MODE;     //�ϵ�Ĭ���ֶ�ģʽ
static volatile  uint32_t g_wheel_status 	= WHEEL_STOP;    	 //�ϵ�Ĭ�ϴ���ɲ��״̬
static volatile  uint32_t g_hcsr04_distance	=0;

#define DEBUG_PRINTF_EN	1

void dgb_printf_safe(const char *format, ...)
{
#if DEBUG_PRINTF_EN	

	va_list args;
	va_start(args, format);
	
	/* ��ȡ�����ź��� */
	xSemaphoreTake(g_mutex_printf,portMAX_DELAY);
	
	vprintf(format, args);
			
	/* �ͷŻ����ź��� */
	xSemaphoreGive(g_mutex_printf);	

	va_end(args);
#else
	(void)0;
#endif
}
int main(void)
{
	/* ����ϵͳ�ж����ȼ�����4 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	/* ��ʼ����ʱ���� */
	delay_init(168);								
	
	/* ��ʼ������1 */
	uart_init(115200);     

	xTaskCreate((TaskFunction_t )app_task_init,             /* ������ں��� */
				(const char*    )"app_task_init",									/* �������� */
				(uint16_t       )512,															/* ����ջ��С */		
				(void*          )NULL,														/* ������ں������� */		
				(UBaseType_t    )6,																/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_init_handle);					/* ������ƿ�ָ�� */ 
		
	/* ����������� */
	vTaskStartScheduler();
	
	while(1);
}
	
void app_task_init(void *pvParameters)
{
	/* �����������ź��� */	  
	g_mutex_printf=xSemaphoreCreateMutex();	
	g_mutex_oled=xSemaphoreCreateMutex();	

	/* ������Ϣ���� */		  
	g_queue_bluetooth = xQueueCreate(QUEUE_BLUETOOTH_LEN , QUEUE_BLUETOOTH_SIZE);
	g_queue_sg90 = xQueueCreate(QUEUE_SG90_LEN,sizeof(uint8_t));
    g_queue_hcsr04 = xQueueCreate(QUEUE_HCSR04_LEN,sizeof(uint8_t));
    g_queue_beep = xQueueCreate(QUEUE_BEEP_LEN,sizeof(uint8_t));
    g_queue_oled = xQueueCreate(QUEUE_OLED_LEN,sizeof(oled_t));
	
	/* �����������ź��� */	  
	g_sem_sg90 =xSemaphoreCreateCounting(255,0);	
	g_sem_hcsr04 =xSemaphoreCreateCounting(255,0);
	g_sem_beep =xSemaphoreCreateCounting(255,0);	
	/* �����¼���־�� */
	g_event_group=xEventGroupCreate();

	xTaskCreate((TaskFunction_t )app_task_beep,             /* ������������ں��� */
				(const char*    )"beep",			/* �������� */
				(uint16_t       )128,						/* ����ջ��С */		
				(void*          )NULL,						/* ������ں������� */		
				(UBaseType_t    )6,							/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_beep_handle);	/* ������ƿ�ָ�� */ 

	xTaskCreate((TaskFunction_t )app_task_rtc,  			/* RTC������ں��� */
				(const char*    )"rtc",				/* �������� */
				(uint16_t       )128,  						/* ����ջ��С */
				(void*          )NULL,						/* ������ں������� */
				(UBaseType_t    )6, 							/* ��������ȼ� */
				(TaskHandle_t*  )&app_task_rtc_handle);		/* ������ƿ�ָ�� */

	xTaskCreate((TaskFunction_t )app_task_dht11,  			/* DHT11������ں��� */
				(const char*    )"dht11",				/* �������� */
				(uint16_t       )128,  						/* ����ջ��С */
				(void*          )NULL,						/* ������ں������� */
				(UBaseType_t    )7, 							/* ��������ȼ� */
				(TaskHandle_t*  )&app_task_dht11_handle);		/* ������ƿ�ָ�� */

	xTaskCreate((TaskFunction_t )app_task_sg90,             /* ���������ں��� */
				(const char*    )"sg90",			/* �������� */
				(uint16_t       )128,						/* ����ջ��С */		
				(void*          )NULL,						/* ������ں������� */		
				(UBaseType_t    )6,							/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_sg90_handle);	/* ������ƿ�ָ�� */ 	

	xTaskCreate((TaskFunction_t )app_task_oled,             /* OLED��Ļ������ں��� */
				(const char*    )"oled",			/* �������� */
				(uint16_t       )512,						/* ����ջ��С */		
				(void*          )NULL,						/* ������ں������� */		
				(UBaseType_t    )7,							/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_oled_handle);	/* ������ƿ�ָ�� */ 

	xTaskCreate((TaskFunction_t )app_task_hcsr04,           /* ������������ں��� */
				(const char*    )"hcsr04",			/* �������� */
				(uint16_t       )128,						/* ����ջ��С */		
				(void*          )NULL,						/* ������ں������� */		
				(UBaseType_t    )7,							/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_hcsr04_handle);	/* ������ƿ�ָ�� */ 	

	xTaskCreate((TaskFunction_t )app_task_bluetooth,           /* ����������ں��� */
				(const char*    )"bluetooth",			/* �������� */
				(uint16_t       )128,						/* ����ջ��С */		
				(void*          )NULL,						/* ������ں������� */		
				(UBaseType_t    )6,							/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_bluetooth_handle);	/* ������ƿ�ָ�� */ 

	xTaskCreate((TaskFunction_t )app_task_l298n_wheel,           /* L298N(����)������ں��� */
				(const char*    )"l298n_wheel",			/* �������� */
				(uint16_t       )512,						/* ����ջ��С */		
				(void*          )NULL,						/* ������ں������� */		
				(UBaseType_t    )7,							/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_l298n_wheel_handle);	/* ������ƿ�ָ�� */ 

	xTaskCreate((TaskFunction_t )app_task_mpu6050,           /* mpu6050������ں��� */
				(const char*    )"mpu6050",			/* �������� */
				(uint16_t       )512,						/* ����ջ��С */		
				(void*          )NULL,						/* ������ں������� */		
				(UBaseType_t    )6,							/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_mpu6050_handle);	/* ������ƿ�ָ�� */ 
		  
	xTaskCreate((TaskFunction_t )app_task_mfrc522,           /* MFRX522������ں��� */
				(const char*    )"mfrc522",			/* �������� */
				(uint16_t       )512,						/* ����ջ��С */		
				(void*          )NULL,						/* ������ں������� */		
				(UBaseType_t    )6,							/* ��������ȼ� */					
				(TaskHandle_t*  )&app_task_mfrc522_handle);	/* ������ƿ�ָ�� */ 

	//xTaskCreate((TaskFunction_t )app_task_cpu_status,  		/* CPUʹ����������ں��� */
	//		  (const char*    )"cpu_status",			/* �������� */
	//		  (uint16_t       )512,  				/* ����ջ��С */
	//		  (void*          )NULL,				/* ������ں������� */
	//		  (UBaseType_t    )6, 					/* ��������ȼ� */
	//		  (TaskHandle_t*  )&app_task_cpu_status_handle);	/* ������ƿ�ָ�� */ 

	/* �������������ʱ�� */
	soft_timer_Handle=xTimerCreate(	(const char*		)"AutoReloadTimer",
									(TickType_t			)1000,/* ��ʱ������ 1000(tick) */
									(UBaseType_t		)pdTRUE,/* ����ģʽ */
									(void*				)1,/* Ϊÿ����ʱ������һ��������ΨһID */
									(TimerCallbackFunction_t)soft_timer_callback); 	
	/* �������������ʱ�� */							
	xTimerStart(soft_timer_Handle,0);

	l298n_wheel_config(30);  		//L298N��ʼ��(�����ĸ����),�ٶ�Ϊ20%(����100)

	/* ��ʼ��OLED */      
	OLED_Init();
	OLED_Clear();
	/* ��ʾlogo */
	OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_logo);									
	/* ����2�� */
	vTaskDelay(2000);
	OLED_Clear();

	/* ��ʾ������С���� */
	OLED_ShowCHinese(32,0,0);
	OLED_ShowCHinese(48,0,1);
	OLED_ShowCHinese(64,0,2);	
	OLED_ShowCHinese(80,0,3);		

	/*  ��ʾ��GZ2132" */
	OLED_ShowString(16,2,(u8 *)"GZ2422",16);

	/* ��ʾ���ȴ��С� */
	OLED_ShowCHinese(64,2,4);
	OLED_ShowCHinese(80,2,5);
	OLED_ShowCHinese(96,2,6);	

	/*  ��ʾ"2021/8/18" */
	OLED_ShowString(32,4,(u8 *)"2024/8/18",16);	
	vTaskDelay(3000);
	OLED_Clear();

	/* ��ǰ���ֹ��ʼ���׶ε����ƽƮ������ */
	bluetooth_config(115200);    //������ʼ��
    /*  ��ѯ��ǰ��λ�Ƿ��ǿ��Ź�����  */
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST)==SET)
	{
		usart3_send_str((u8 *)"iwdg reset cpu\r\n");
	}
	else
	{
		usart3_send_str((u8 *)"normal reset cpu\r\n");
	}RCC_ClearFlag();
	LED_Init();
	beep_config();  		//��������ʼ��
	sg90_config(); 			//�����ʼ��
	hcsr04_config();  	    //��������ʼ��
	dht11_config();         //��ʪ�ȳ�ʼ��

	MPU_Init();
	mpu_dmp_init();

	rtc_config();			//RTCʱ�ӳ�ʼ��
	MFRC522_Initializtion();  //MFRC522��ʼ��
	/* �������Ź���ʼ�� */		
	iwdg_init();        //��ʱʱ��2s
	
	vTaskDelete(NULL);		
}

static void app_task_mfrc522(void* pvParameters)   	//����	mfrc522
{
	uint8_t  	card_pydebuf[2]={0};
	uint8_t  	card_numberbuf[5]={0};  //��ID
	oled_t		oled;		
	BaseType_t 	xReturn=pdFALSE;
	uint8_t 	ID_buff[5]={0};
	uint8_t     beep_sta=0;

	while(1)
	{
		MFRC522_Initializtion();
		if( 0==MFRC522_Request(0x52, card_pydebuf))			//Ѱ��
		{
			MFRC522_Anticoll(card_numberbuf);			//��ײ����

		    sprintf((char*)ID_buff,"%02X%02X%02X%02X%02X",card_numberbuf[0],card_numberbuf[1],card_numberbuf[2],card_numberbuf[3],card_numberbuf[4]);
			dgb_printf_safe("%s\r\n",ID_buff);	//����

			if(strstr((char*)ID_buff,"46256BAFA7"))   //����
			{
				/*  ��������ֻ����mfrc522�����Լ�OLED����  */
				//vTaskSuspend(app_task_beep_handle);
				vTaskSuspend(app_task_rtc_handle);
				vTaskSuspend(app_task_dht11_handle);
				vTaskSuspend(app_task_sg90_handle);
				vTaskSuspend(app_task_hcsr04_handle);
				vTaskSuspend(app_task_bluetooth_handle);
				vTaskSuspend(app_task_l298n_wheel_handle);
				vTaskSuspend(app_task_mpu6050_handle);
				USART_Cmd(USART3,DISABLE);
				USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);

				oled.ctrl=OLED_CTRL_SHOW_PICTURE;
				oled.x=40;
				oled.y=0;
				oled.pic_width=50;
				oled.pic_height=8;				
				oled.pic=pic_locker_icon;
				
				OLED_Clear();
				xReturn = xQueueSend( 	g_queue_oled,/* ��Ϣ���еľ�� */
										&oled,		/* ���͵���Ϣ���� */
										100);		/* �ȴ�ʱ�� 100 Tick */
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_key] xQueueSend oled picture error code is %d\r\n",xReturn);

				//��������һ��
				beep_sta=2;
				xReturn = xQueueSend( g_queue_beep,/* ��Ϣ���еľ�� */
									  &beep_sta,	/* ���͵���Ϣ���� */
									  0);		/* �ȴ�ʱ�� 100 Tick */			
				if(xReturn != pdPASS)
				{
					dgb_printf_safe("[app_task_mpu6050] xQueueSend beep_sta error code is %d\r\n",xReturn);
					continue;
				}
				/* �����ȴ��ź���������ȷ��������ɶԷ������Ŀ��� */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);	
			}
			else if(strstr((char*)ID_buff,"C9F3034D74"))  //����
			{
				OLED_Clear();	
				//��������һ��
				beep_sta=2;
				xReturn = xQueueSend( g_queue_beep,/* ��Ϣ���еľ�� */
									  &beep_sta,	/* ���͵���Ϣ���� */
									  0);		/* �ȴ�ʱ�� 100 Tick */			
				if(xReturn != pdPASS)
				{
					dgb_printf_safe("[app_task_mpu6050] xQueueSend beep_sta error code is %d\r\n",xReturn);
					continue;
				}
				/* �����ȴ��ź���������ȷ��������ɶԷ������Ŀ��� */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);	

				//vTaskResume(app_task_beep_handle);
				vTaskResume(app_task_rtc_handle);
				vTaskResume(app_task_dht11_handle);
				vTaskResume(app_task_sg90_handle);
				vTaskResume(app_task_hcsr04_handle);
				vTaskResume(app_task_bluetooth_handle);
				vTaskResume(app_task_l298n_wheel_handle);
				vTaskResume(app_task_mpu6050_handle);
				USART_Cmd(USART3,ENABLE);
				USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
			}
		}
		delay_ms(1000);
	}
}

static void app_task_rtc(void* pvParameters)  //���� RTC
{
	uint8_t 		buf[16]={0};	
	oled_t			oled;
	BaseType_t		xReturn;
	EventBits_t 	EventValue;
	RTC_TimeTypeDef RTC_TimeStructure;

	dgb_printf_safe("[app_task_rtc] create success\r\n");		
	
	for(;;)
	{
		/* �ȴ��¼����е���Ӧ�¼�λ����ͬ�� */
		EventValue=xEventGroupWaitBits((EventGroupHandle_t	)g_event_group,		
									   (EventBits_t			)EVENT_GROUP_RTC_WAKEUP,
									   (BaseType_t			)pdTRUE,				
									   (BaseType_t			)pdFALSE,
									   (TickType_t			)portMAX_DELAY);
		if(EventValue & EVENT_GROUP_RTC_WAKEUP)
		{
			/* RTC_GetTime����ȡʱ�� */
			RTC_GetTime(RTC_Format_BCD, &RTC_TimeStructure); 
				
			/* ��ʽ���ַ��� */
			sprintf((char *)buf,"Run: %02x:%02x:%02x",RTC_TimeStructure.RTC_Hours,RTC_TimeStructure.RTC_Minutes,RTC_TimeStructure.RTC_Seconds);
			
			/* oled��ʾʱ�� */
			oled.ctrl=OLED_CTRL_SHOW_STRING;
			oled.x=8;
			oled.y=0;
			oled.str=buf;
			oled.font_size=16;

			xReturn = xQueueSend( 	g_queue_oled,/* ��Ϣ���еľ�� */
									&oled,	/* ���͵���Ϣ���� */
									100);		/* �ȴ�ʱ�� 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_rtc] xQueueSend oled string error code is %d\r\n",xReturn);			
			
			dgb_printf_safe("[app_task_rtc] %s\r\n",buf);			
			
		}
	}
} 

static void app_task_dht11(void* pvParameters)
{
	uint8_t 	dht11_data[5]={0};
	uint8_t 	buf[16]={0};
	oled_t		oled;
	BaseType_t	xReturn;
	int8_t 		res=0;
	dgb_printf_safe("[app_task_dht] create success\r\n");	
	
	for(;;)
	{	
		if(res == dht11_read(dht11_data))
		{
			sprintf((char *)buf,"T:%02d.%d,H:%02d.%d",dht11_data[2],dht11_data[3],dht11_data[0],dht11_data[1]);
		
			oled.ctrl=OLED_CTRL_SHOW_STRING;
			oled.x=8;
			oled.y=2;
			oled.str=buf;
			oled.font_size=16;

			xReturn = xQueueSend( 	g_queue_oled,/* ��Ϣ���еľ�� */
									&oled,		/* ���͵���Ϣ���� */
									100);		/* �ȴ�ʱ�� 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_dht] xQueueSend oled string error code is %d\r\n",xReturn);				
			
			dgb_printf_safe("[app_task_dht] %s\r\n",buf);		
		}
		else
		{
			dgb_printf_safe("[error] %d\r\n",res);		
		}
		vTaskDelay(6000);
	}
}

static void app_task_oled(void *pvParameters)    		 //����  oled��Ļ
{
	oled_t oled;
	BaseType_t xReturn=pdFALSE;	
	dgb_printf_safe("app_task_oled success\r\n");
	while(1)
	{
		xReturn = xQueueReceive( g_queue_oled,	/* ��Ϣ���еľ�� */
								 &oled, 			/* �õ�����Ϣ���� */
								 portMAX_DELAY);	/* �ȴ�ʱ��һֱ�� */
		if(xReturn != pdPASS)
			continue;

		switch(oled.ctrl)
		{
			case OLED_CTRL_SHOW_STRING:
			{
				/* ��ʾ�ַ��� */
				OLED_ShowString(oled.x,
								oled.y,
								oled.str,
								oled.font_size);
			}break;

			case OLED_CTRL_SHOW_PICTURE:
			{
				/* ��ʾͼƬ */
				OLED_DrawBMP(	oled.x,
								oled.y,
								oled.x+oled.pic_width,
								oled.y+oled.pic_height,
								(unsigned char*)oled.pic);
			}break;

			default:dgb_printf_safe("[app_task_oled] oled ctrl code is invalid\r\n");	
				break;
		}				
	}
}

static void app_task_beep(void *pvParameters)    //����������
{
	uint8_t 	beep_sta=0;	
	BaseType_t	xReturn	=pdFALSE;	
	dgb_printf_safe("app_task_beep success\r\n");
		
	for(;;)
	{
		xReturn = xQueueReceive( g_queue_beep,	/* ��Ϣ���еľ�� */
								 &beep_sta, 		/* �õ�����Ϣ���� */
								portMAX_DELAY);	/* �ȴ�ʱ��һֱ�� */
		if(xReturn != pdPASS)
			continue;
		
		if(beep_sta==0)
			BEEP(0);
		else if(beep_sta==1)
			BEEP(1);
		else if(beep_sta==2)
		{
			BEEP(1);
			delay_ms(100);
			BEEP(0);
		}	

		/* �ͷ��ź��������߶Է�����ǰbeep���������Ѿ���� */
		xSemaphoreGive(g_sem_beep);	
		delay_ms(100);	
	}
}


static void app_task_mpu6050(void *pvParameters)       //����mpu6050
{
	
	uint8_t 	buf[32]={0};	
	float       pitch,roll,yaw; 		//ŷ����
	uint8_t     beep_sta;
	oled_t		oled;		
	BaseType_t 	xReturn=pdFALSE;
	dgb_printf_safe("app_task_mpu6050 success\r\n");

	while(1)
	{
		if(0==mpu_dmp_get_data(&pitch,&roll,&yaw))
		{
			if(yaw>0)	
				sprintf((char *)buf,"turn left:%4.1f",yaw);
			else if(yaw<0)
				sprintf((char *)buf,"turn right:%4.1f",yaw*(-1));

			oled.ctrl=OLED_CTRL_SHOW_STRING;
			oled.x=0;
			oled.y=4;
			oled.str=buf;
			oled.font_size=16;
			xReturn = xQueueSend( 	g_queue_oled,/* ��Ϣ���еľ�� */
									&oled,	/* ���͵���Ϣ���� */
									0);		/* �ȴ�ʱ�� 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_mpu6050] xQueueSend oled string error code is %d\r\n",xReturn);

			
			if(roll<=-60 || roll>=60 ||pitch<=-60 || pitch>=60)
			{
				//BEEP(1);
				beep_sta=1;
				xReturn = xQueueSend( g_queue_beep,/* ��Ϣ���еľ�� */
									  &beep_sta,	/* ���͵���Ϣ���� */
									  0);		/* �ȴ�ʱ�� 100 Tick */			
			    if(xReturn != pdPASS)
				{
					dgb_printf_safe("[app_task_mpu6050] xQueueSend beep_sta error code is %d\r\n",xReturn);
					continue;
				}
				/* �����ȴ��ź���������ȷ��������ɶԷ������Ŀ��� */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);		
			}else
			{
				//BEEP(0);
				beep_sta=0;
				xReturn = xQueueSend( g_queue_beep,/* ��Ϣ���еľ�� */
									  &beep_sta,	/* ���͵���Ϣ���� */
									  0);		/* �ȴ�ʱ�� 100 Tick */			
				if(xReturn != pdPASS)
				{
					dgb_printf_safe("[app_task_mpu6050] xQueueSend beep_sta error code is %d\r\n",xReturn);
					continue;
				}
				/* �����ȴ��ź���������ȷ��������ɶԷ������Ŀ��� */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);					
			}
		}	
		delay_ms(100);
	}		
}

static void app_task_sg90(void *pvParameters)   //�������
{
	uint8_t		 sg90_sta=0;
	BaseType_t 	 xReturn=pdFALSE;	

	dgb_printf_safe("app_task_sg90 success\r\n");

	while(1)
	{
		xReturn = xQueueReceive( g_queue_sg90,	    /* ��Ϣ���еľ�� */
								 &sg90_sta, 		/* �õ�����Ϣ���� */
								 portMAX_DELAY);	/* �ȴ�ʱ��һֱ�� */
		if(xReturn != pdPASS)
			continue;

		if(sg90_sta==170)  //������ת      
		{
			sg90_angle(170);  //170�� 
			delay_ms(1000);  			
		}		
		else if(sg90_sta==0)  //������ת      
		{
			sg90_angle(0);  //0�� 
			delay_ms(1000);  
		}
		else if(sg90_sta==90)  //��������   ��������
		{
			sg90_angle(90);  //90�� 
			delay_ms(1000);  
		}	
	
		xSemaphoreGive(g_sem_sg90);  /* �ͷ��ź��������߶Է�����ǰsg90���������Ѿ���� */

		/*   0�����̬��|     */

		//TIM_SetCompare3(TIM2,185);//90��   ---   
		//sg90_angle(90);	 //90��   --- 	
		//delay_ms(1000); 		
		
		//TIM_SetCompare3(TIM2,180);//135�� 	////
		//sg90_angle(170);  //135�� 	////
		//delay_ms(1000); 
		
		//sg90_angle(135);  //135�� 	////
		//delay_ms(1000); 

		//TIM_SetCompare3(TIM2,185);//90��   ---  
		//sg90_angle(90);	//90��   ---  	 
		//delay_ms(1000); 

		//TIM_SetCompare3(TIM2,190);//45��   \\\\  	
		//sg90_angle(0);  //45��   \\\\ 
		//delay_ms(1000); 
	}
}


static void app_task_l298n_wheel(void *pvParameters)   //����  L298N(����)
{	
	uint8_t 	sg90_sta=0;
	uint8_t 	hcsr04_sta=0;
	uint32_t    left_distance=0;
	uint32_t    right_distance=0;

	//BaseType_t 	xReturn=pdFALSE;

	dgb_printf_safe("app_task_l298n_wheel success\r\n");

	while(1)
	{
		if(g_wheel_mode == AVOID_MODE)     //����ģʽ
		{

			/*  ���ƶ���ص�Ĭ��λ������  */
			//sg90_sta=90;   //�ص�����90��
			//xReturn = xQueueSend( g_queue_sg90,/* ��Ϣ���еľ�� */
			//						&sg90_sta,	/* ���͵���Ϣ���� */
			//						0);		/* �ȴ�ʱ�� 100 Tick */
			///* �����ȴ��ź���������ȷ��������ɶԶ���Ŀ��� */
			//xReturn=xSemaphoreTake(g_sem_sg90,portMAX_DELAY);

			//����������������ȡ��������
			hcsr04_sta=1;
			xQueueSend( g_queue_hcsr04,/* ��Ϣ���еľ�� */
								  &hcsr04_sta,	/* ���͵���Ϣ���� */
								  0);		/* �ȴ�ʱ�� 100 Tick */	
			/* �����ȴ��ź���������ȷ��������ɶԳ������������ȡ�������� */
			xSemaphoreTake(g_sem_hcsr04,portMAX_DELAY);

			if(g_hcsr04_distance<250)  //����̫��
			{
				/*    ���˲���    */
				LEFT_IN1 = 1;      
				LEFT_IN2 = 0;		
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;	
				//dgb_printf_safe("����");
				delay_ms(900);
				/*    ɲ������    */
				LEFT_IN1=1;      
				LEFT_IN2=1;
				RIGHT_IN1=1;
				RIGHT_IN2=1;	
				//dgb_printf_safe("ɲ��");

				/*  ���ƶ����ת  */
				sg90_sta=170;   //��ת��170��(��ת��180�����Ῠ����)
				xQueueSend( g_queue_sg90,/* ��Ϣ���еľ�� */
									  &sg90_sta,	/* ���͵���Ϣ���� */
									  0);		/* �ȴ�ʱ�� 100 Tick */
				/* �����ȴ��ź���������ȷ��������ɶԶ���Ŀ��� */
				xSemaphoreTake(g_sem_sg90,portMAX_DELAY);
				//dgb_printf_safe("�����ת�ɹ�\r\n");

				//����������������ȡ��ת�����������
				hcsr04_sta=1;
				xQueueSend( g_queue_hcsr04,/* ��Ϣ���еľ�� */
									&hcsr04_sta,	/* ���͵���Ϣ���� */
									0);		/* �ȴ�ʱ�� 100 Tick */	
				/* �����ȴ��ź���������ȷ��������ɶԳ������������ȡ�������� */
				xSemaphoreTake(g_sem_hcsr04,portMAX_DELAY);
				left_distance = g_hcsr04_distance; //��̽�⵽��߾���浽left����
				//dgb_printf_safe("���������ɹ�\r\n");//���Գ������ɹ�

				/*  ���ƶ����ת  */
				sg90_sta=0;   //��ת��0��
				xQueueSend( g_queue_sg90,/* ��Ϣ���еľ�� */
									  &sg90_sta,	/* ���͵���Ϣ���� */
									  0);		/* �ȴ�ʱ�� 100 Tick */
				/* �����ȴ��ź���������ȷ��������ɶԶ���Ŀ��� */
				xSemaphoreTake(g_sem_sg90,portMAX_DELAY);
				//dgb_printf_safe("�����ת�ɹ�\r\n");//����

				delay_ms(70);   //���������70ms��,��ֹ�����źŸ���
				//����������������ȡ��ת�����������
				hcsr04_sta=1;
				xQueueSend( g_queue_hcsr04,/* ��Ϣ���еľ�� */
									&hcsr04_sta,	/* ���͵���Ϣ���� */
									0);		/* �ȴ�ʱ�� 100 Tick */	
				/* �����ȴ��ź���������ȷ��������ɶԳ������������ȡ�������� */
				xSemaphoreTake(g_sem_hcsr04,portMAX_DELAY);
				right_distance = g_hcsr04_distance; //��̽�⵽�ұ߾���浽right����
				//dgb_printf_safe("���������ɹ�\r\n");//���Գ������ɹ�
				
				/*  ���ƶ���ص�Ĭ��λ������  */
				sg90_sta=90;   //�ص�����90��
				xQueueSend( g_queue_sg90,/* ��Ϣ���еľ�� */
									  &sg90_sta,	/* ���͵���Ϣ���� */
									  0);		/* �ȴ�ʱ�� 100 Tick */
				/* �����ȴ��ź���������ȷ��������ɶԶ���Ŀ��� */
				xSemaphoreTake(g_sem_sg90,portMAX_DELAY);

				/*  ����������ұ�   */
				if(left_distance>right_distance)  
				{
					/*  ��ת����  */
					LEFT_IN1=1;      //��������;
					LEFT_IN2=0;
					RIGHT_IN1=0;     //�ҵ��ǰ��
					RIGHT_IN2=1;
					//dgb_printf_safe("��ת");

					delay_ms(500);  //��ʱ550ms���´λص���ȫ����ʵ��ǰ����������ʵ��90����ת
				}
				else    /*  �Ҳ����������   */
				{
					/*  ��ת����  */
					LEFT_IN1 = 0;      //����ǰ��;
					LEFT_IN2 = 1;
					RIGHT_IN1 = 1;     //�ҵ������
					RIGHT_IN2 = 0;
					//dgb_printf_safe("��ת");

					delay_ms(500); //��ʱ550ms���´λص���ȫ����ʵ��ǰ����������ʵ��90����ת
				}
			}
			else      //���밲ȫ
			{
				/*  ǰ������  */
				LEFT_IN1 =0;    //����ǰ��
				LEFT_IN2 =1;
				RIGHT_IN1=0;    //�ҵ��ǰ��
				RIGHT_IN2=1;
				//dgb_printf_safe("ǰ��\r\n");				
			}
		}
		else if(g_wheel_mode == FOLLOW_MODE)      //����ģʽ
		{
			hcsr04_sta=1;
			xQueueSend( g_queue_hcsr04,/* ��Ϣ���еľ�� */
								  &hcsr04_sta,	/* ���͵���Ϣ���� */
								  0);		/* �ȴ�ʱ�� 100 Tick */	
			/* �����ȴ��ź���������ȷ��������ɶԳ������������ȡ�������� */
			xSemaphoreTake(g_sem_hcsr04,portMAX_DELAY);	
			//dgb_printf_safe("���������ɹ�\r\n"); //��ʽ���������ɹ�;

			if(g_hcsr04_distance>1000)      //����100cm��û����(��ֹͣ)
			{
				/*  ɲ������  */
				LEFT_IN1=1;     
				LEFT_IN2=1;
				RIGHT_IN1=1;
				RIGHT_IN2=1;
				//dgb_printf_safe("ɲ��\r\n");	
			}
			else if(g_hcsr04_distance>=400&&g_hcsr04_distance<1000)  //��40cm-100cm��Χ����(��ǰ��)
			{				
				/*  ǰ������  */
				LEFT_IN1 =0;    
				LEFT_IN2 =1;
				RIGHT_IN1=0;    
				RIGHT_IN2=1;	
				//dgb_printf_safe("ǰ��\r\n");		
			}
			else if(g_hcsr04_distance<400 && g_hcsr04_distance>=200)   //20cm-40cm��Χ����(��ɲ��)
			{
				/*  ɲ������  */
				LEFT_IN1=1;      
				LEFT_IN2=1;
				RIGHT_IN1=1;
				RIGHT_IN2=1;
				//dgb_printf_safe("ɲ��\r\n");	
			}
			else if(g_hcsr04_distance<200)    //��20cm������(�����ֱ����ȫ����20cm-40cm��Χ�ڷ���ɲ��)
			{
				/*  ���˲���  */
				LEFT_IN1 = 1;      
				LEFT_IN2 = 0;		
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;	
				//dgb_printf_safe("����\r\n");	
			}				
		}
		else if(g_wheel_mode == MANUAL_MODE)    //�ֶ�ģʽ
		{
			if(g_wheel_status == WHEEL_STOP)      //ɲ��
			{
				/*  ɲ������  */
				LEFT_IN1=1;      
				LEFT_IN2=1;
				RIGHT_IN1=1;
				RIGHT_IN2=1;
				//dgb_printf_safe("ɲ��\r\n");	
			}
			else if(g_wheel_status == WHEEL_UP)   //ǰ��
			{
				/*  ǰ������  */
				LEFT_IN1 =0;    
				LEFT_IN2 =1;
				RIGHT_IN1=0;    
				RIGHT_IN2=1;	

				//dgb_printf_safe("ǰ��");

			}
			else if(g_wheel_status == WHEEL_DOWN)  //����
			{
				/*  ���˲���  */
				LEFT_IN1 = 1;      
				LEFT_IN2 = 0;		
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;	
				//dgb_printf_safe("����\r\n");	
			}
			else if(g_wheel_status == WHEEL_LEFT)  //90����ת
			{
				/*  ��ת����  */
				LEFT_IN1=1;      
				LEFT_IN2=0;
				RIGHT_IN1=0;     
				RIGHT_IN2=1;
				//dgb_printf_safe("��ת");

				delay_ms(500);

				g_wheel_status = WHEEL_UP;  //��ʱ550ms���Ϊǰ��״̬����ʵ��90����ת
			}
			else if(g_wheel_status == WHEEL_RIGHT)  //90����ת
			{
				/*  ��ת����  */
				LEFT_IN1 = 0;      
				LEFT_IN2 = 1;
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;
				//dgb_printf_safe("��ת");

				delay_ms(500);
				g_wheel_status = WHEEL_UP;  //��ʱ550ms���Ϊǰ��״̬����ʵ��90����ת

			}
			else if(g_wheel_status == WHEEL_ORIGIN_LEFT)  //ԭ����ת
			{
				/*  ԭ����ת����  */
				LEFT_IN1=1;      
				LEFT_IN2=0;
				RIGHT_IN1=0;     
				RIGHT_IN2=1;
				//dgb_printf_safe("ԭ����ת\r\n");
			}
			else if(g_wheel_status == WHEEL_ORIGIN_RIGHT)  //ԭ����ת 
			{
				/*  ԭ����ת����  */
				LEFT_IN1 = 0;      
				LEFT_IN2 = 1;
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;
				//dgb_printf_safe("ԭ����ת\r\n");
			}			
		}
		else     //����ֹͣģʽ
		{
			LEFT_IN1=1;      
			LEFT_IN2=1;
			RIGHT_IN1=1;
			RIGHT_IN2=1;
			//dgb_printf_safe("ɲ��\r\n");

			/*  ���ƶ���ص�Ĭ��λ������  */
			sg90_sta=90;   //�ص�����90��
			xQueueSend( g_queue_sg90,/* ��Ϣ���еľ�� */
									&sg90_sta,	/* ���͵���Ϣ���� */
									0);		/* �ȴ�ʱ�� 100 Tick */
			/* �����ȴ��ź���������ȷ��������ɶԶ���Ŀ��� */
			xSemaphoreTake(g_sem_sg90,portMAX_DELAY);		
		    dgb_printf_safe("����ص���ʼλ��\r\n");//����
		}
		delay_ms(10);
	}
}

static void app_task_hcsr04(void *pvParameters)    //����  ���������
{
	uint8_t hcsr04_sta=0;
	BaseType_t xReturn=pdFALSE;	

	dgb_printf_safe("app_task_hcsr04 success\r\n");
	while(1)
	{
		xReturn = xQueueReceive( g_queue_hcsr04,	    /* ��Ϣ���еľ�� */
								 &hcsr04_sta, 		/* �õ�����Ϣ���� */
								 portMAX_DELAY);	/* �ȴ�ʱ��һֱ�� */
		if(xReturn != pdPASS)
			continue;
		
		if(hcsr04_sta)
		{
			g_hcsr04_distance=hcsr04_get_distance();
			//����쳣
			//if(g_hcsr04_distance<20 || g_hcsr04_distance>4000)
			//{
			//	continue;
			//}
			//dgb_printf_safe("distance = %dmm\r\n",g_hcsr04_distance);
			xSemaphoreGive(g_sem_hcsr04);  /* �ͷ��ź��������߶Է�����ǰsg90���������Ѿ���� */

		}
	}
}

static void app_task_bluetooth(void *pvParameters)    //����  ����
{
	static  uint8_t  bluetooth_recv_buf[QUEUE_BLUETOOTH_SIZE]={0};

	BaseType_t xReturn=pdFALSE;	

	dgb_printf_safe("app_task_bluetooth success\r\n");

	while(1)
	{
		xReturn = xQueueReceive( g_queue_bluetooth,  /* ��Ϣ���еľ�� */
								 bluetooth_recv_buf, /* �õ�����Ϣ���� */
								 portMAX_DELAY);     /* �ȴ�ʱ��һֱ�� */		

		if(xReturn != pdPASS)
		{
			dgb_printf_safe("[app_task_bluetooth] xQueueReceive bluetooth_recv_buf error code is %d\r\n",xReturn);
			
			continue;
		}
		dgb_printf_safe("%s\r\n",bluetooth_recv_buf);  //����	
		if(strstr((char *)bluetooth_recv_buf,"avoid_mode#"))          //����ģʽ
		{
			/* �ָ�����������Ͷ������ */
			//vTaskResume(app_task_hcsr04_handle);
			//vTaskResume(app_task_sg90_handle);	

			g_wheel_mode = AVOID_MODE;
		} 
		else if(strstr((char *)bluetooth_recv_buf,"manual_mode#"))    //�ֶ�ģʽ  
		{
			/* �����ò����ĳ���������Ͷ������ */
			//vTaskSuspend(app_task_hcsr04_handle);	
			//vTaskSuspend(app_task_sg90_handle);	

			g_wheel_mode = MANUAL_MODE;

		}
		else if(strstr((char *)bluetooth_recv_buf,"follow_mode#"))    //����ģʽ
		{
			//vTaskSuspend(app_task_sg90_handle);	/* �����ò����Ķ������ */
			//vTaskResume(app_task_hcsr04_handle);/* �ָ����������� */

			g_wheel_mode = FOLLOW_MODE;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"stop_mode#"))    //����ֹͣģʽ
		{
			//vTaskResume(app_task_sg90_handle);	/* �����ò����Ķ������ */
			//vTaskSuspend(app_task_hcsr04_handle);/* �ָ����������� */

			g_wheel_mode = STOP_MODE;
			
		}		
		else if(strstr((char *)bluetooth_recv_buf,"stop#"))           //ɲ��
		{

			g_wheel_status = WHEEL_STOP;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"up#"))             //ǰ��
		{

			g_wheel_status = WHEEL_UP;

		}
		else if(strstr((char *)bluetooth_recv_buf,"down#"))           //����
		{

			g_wheel_status = WHEEL_DOWN;

		}
		/*     ԭ����ת��ԭ����תҪ������ת����ת�ĺ���  */
		else if(strstr((char *)bluetooth_recv_buf,"origin_left#"))			  //ԭ����ת		
		{

			g_wheel_status = WHEEL_ORIGIN_LEFT;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"origin_right#"))          //ԭ����ת
		{

			g_wheel_status = WHEEL_ORIGIN_RIGHT;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"left#"))			  //��ת		
		{

			g_wheel_status = WHEEL_LEFT;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"right#"))          //��ת
		{

			g_wheel_status = WHEEL_RIGHT;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"stop#"))           //ɲ��
		{

			g_wheel_status = WHEEL_STOP;
			
		}

		memset(bluetooth_recv_buf,0,sizeof bluetooth_recv_buf);	  //���bluetooth_recv_buf����

	}
}



static void soft_timer_callback(void* parameter)  //1sι��һ��
{		
	/* �ر��ж� */
	portDISABLE_INTERRUPTS();
	
	/* ι����ˢ���������ֵ */
	IWDG_ReloadCounter();	
	
	//dgb_printf_safe("7");
	/* ���ж� */
	portENABLE_INTERRUPTS();
	
	//TickType_t tick_cnt;
	//tick_cnt = xTaskGetTickCount();	/* ��ȡ�δ�ʱ���ļ���ֵ */
} 

/*************************************************************************
		���±���Ҫ�ṩ�ģ�����FreeRTOSConfig.h
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()    ConfigureTimeForRunTimeStats()//��ʱ��3�ṩʱ��ͳ�Ƶ�ʱ����Ƶ��Ϊ10K��������Ϊ100us
#define portGET_RUN_TIME_COUNTER_VALUE()			FreeRTOSRunTimeTicks		//��ȡʱ��ͳ��ʱ��ֵ	
 *************************************************************************/
#if 0
static void app_task_cpu_status(void *pvParameters)
{
	BaseType_t xReturn=pdFALSE;	
	
	uint8_t cpu_run_info[400];		//������������ʱ����Ϣ
	
	for(;;)
	{
		memset(cpu_run_info,0,400);				//��Ϣ����������
		
		vTaskList((char *)&cpu_run_info);  //��ȡ��������ʱ����Ϣ
		
		printf("---------------------------------------------\r\n");
		printf("������      ����״̬ ���ȼ�   ʣ��ջ �������\r\n");
		printf("%s", cpu_run_info);
		printf("---------------------------------------------\r\n");
		
		memset(cpu_run_info,0,400);				//��Ϣ����������
		
		vTaskGetRunTimeStats((char *)&cpu_run_info);
		
		printf("������       ���м���         ʹ����\r\n");
		printf("%s", cpu_run_info);
		printf("---------------------------------------------\r\n\n");
		vTaskDelay(1000);   /* ��ʱ1000��tick */	
	}
} 

//FreeRTOSʱ��ͳ�����õĽ��ļ�����
volatile unsigned long long FreeRTOSRunTimeTicks;
//��ʼ��TIM3ʹ��ΪFreeRTOS��ʱ��ͳ���ṩʱ��
void ConfigureTimeForRunTimeStats(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	FreeRTOSRunTimeTicks=0;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
	TIM_TimeBaseInitStructure.TIM_Period   = 100; 			//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler= 84-1;  		//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
		FreeRTOSRunTimeTicks++;
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}
#endif
/*-----------------------------------------------------------*/

 void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

 void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

 void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}


 void vApplicationTickHook( void )
{

}
