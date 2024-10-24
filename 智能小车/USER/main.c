#include "include.h"

/* 软件定时器句柄 */
static TimerHandle_t soft_timer_Handle =NULL;  
/* 软件定时器 */
static void soft_timer_callback(void* parameter);   //1s进一次喂狗
//自行增加
static StackType_t  TimerTaskStack[configMINIMAL_STACK_SIZE];
static StaticTask_t TimerTaskTCB;
//定时器任务所需内存
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
	*ppxTimerTaskTCBBuffer=&TimerTaskTCB;
	*ppxTimerTaskStackBuffer=TimerTaskStack; 
	*pulTimerTaskStackSize=configMINIMAL_STACK_SIZE;
}

/* 任务句柄 */ 
static TaskHandle_t app_task_init_handle         = NULL;    //初始化句柄   
static TaskHandle_t app_task_sg90_handle         = NULL;    //舵机句柄 
static TaskHandle_t app_task_oled_handle         = NULL;    //OLED屏幕句柄 
static TaskHandle_t app_task_hcsr04_handle		 	 = NULL;  	//超声波句柄
static TaskHandle_t app_task_beep_handle		 		 = NULL;    //蜂鸣器句柄
static TaskHandle_t app_task_bluetooth_handle		 = NULL;    //蓝牙句柄
static TaskHandle_t app_task_l298n_wheel_handle	 = NULL;    //L298N(轮子)句柄
static TaskHandle_t app_task_mpu6050_handle			 = NULL;    //mpu6050句柄
static TaskHandle_t app_task_rtc_handle					 = NULL;		//RTC句柄
static TaskHandle_t app_task_dht11_handle 			 = NULL;		//DHT11句柄
static TaskHandle_t app_task_mfrc522_handle 	 	 = NULL;		//mfrc522句柄

//static TaskHandle_t app_task_cpu_status_handle = NULL;


/* 任务函数 */
static void app_task_init(void *pvParameters);    			//任务  硬件/任务初始化 
static void app_task_sg90(void *pvParameters);    			//任务  舵机
static void app_task_oled(void *pvParameters);    			//任务  oled屏幕
static void app_task_hcsr04(void *pvParameters);   	 		//任务  超声波测距
static void app_task_beep(void *pvParameters);     			//任务  蜂鸣器
static void app_task_bluetooth(void *pvParameters);     //任务  蓝牙
static void app_task_l298n_wheel(void *pvParameters);   //任务  L298N(轮子)
static void app_task_mpu6050(void *pvParameters);       //任务	mpu6050 
static void app_task_rtc(void* pvParameters);   				//任务 	rtc 
static void app_task_dht11(void* pvParameters);   			//任务	dht11
static void app_task_mfrc522(void* pvParameters);   		//任务	mfrc522

//static void app_task_cpu_status(void *pvParameters);        //任务  查询CPU使用状态


/* 消息队列句柄 */
QueueHandle_t   g_queue_bluetooth;    //蓝牙 
QueueHandle_t   g_queue_sg90;         //舵机
QueueHandle_t   g_queue_hcsr04;       //超声波
QueueHandle_t   g_queue_oled;        	//OLED屏幕
QueueHandle_t   g_queue_beep;        	//蜂鸣器

/* 互斥型信号量句柄 */
SemaphoreHandle_t g_mutex_printf;     //USART1
SemaphoreHandle_t g_mutex_oled;

/* 计数型信号量 */
SemaphoreHandle_t g_sem_sg90;
SemaphoreHandle_t g_sem_beep;
SemaphoreHandle_t g_sem_hcsr04;
SemaphoreHandle_t g_sem_oled;
/* 事件标志组句柄 */
EventGroupHandle_t g_event_group;	

/* 全局变量 */ 
static volatile  uint32_t g_wheel_mode 	 	= MANUAL_MODE;     //上电默认手动模式
static volatile  uint32_t g_wheel_status 	= WHEEL_STOP;    	 //上电默认处于刹车状态
static volatile  uint32_t g_hcsr04_distance	=0;

#define DEBUG_PRINTF_EN	1

void dgb_printf_safe(const char *format, ...)
{
#if DEBUG_PRINTF_EN	

	va_list args;
	va_start(args, format);
	
	/* 获取互斥信号量 */
	xSemaphoreTake(g_mutex_printf,portMAX_DELAY);
	
	vprintf(format, args);
			
	/* 释放互斥信号量 */
	xSemaphoreGive(g_mutex_printf);	

	va_end(args);
#else
	(void)0;
#endif
}
int main(void)
{
	/* 设置系统中断优先级分组4 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	/* 初始化延时函数 */
	delay_init(168);								
	
	/* 初始化串口1 */
	uart_init(115200);     

	xTaskCreate((TaskFunction_t )app_task_init,             /* 任务入口函数 */
				(const char*    )"app_task_init",									/* 任务名字 */
				(uint16_t       )512,															/* 任务栈大小 */		
				(void*          )NULL,														/* 任务入口函数参数 */		
				(UBaseType_t    )6,																/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_init_handle);					/* 任务控制块指针 */ 
		
	/* 开启任务调度 */
	vTaskStartScheduler();
	
	while(1);
}
	
void app_task_init(void *pvParameters)
{
	/* 创建互斥型信号量 */	  
	g_mutex_printf=xSemaphoreCreateMutex();	
	g_mutex_oled=xSemaphoreCreateMutex();	

	/* 创建消息队列 */		  
	g_queue_bluetooth = xQueueCreate(QUEUE_BLUETOOTH_LEN , QUEUE_BLUETOOTH_SIZE);
	g_queue_sg90 = xQueueCreate(QUEUE_SG90_LEN,sizeof(uint8_t));
    g_queue_hcsr04 = xQueueCreate(QUEUE_HCSR04_LEN,sizeof(uint8_t));
    g_queue_beep = xQueueCreate(QUEUE_BEEP_LEN,sizeof(uint8_t));
    g_queue_oled = xQueueCreate(QUEUE_OLED_LEN,sizeof(oled_t));
	
	/* 创建计数型信号量 */	  
	g_sem_sg90 =xSemaphoreCreateCounting(255,0);	
	g_sem_hcsr04 =xSemaphoreCreateCounting(255,0);
	g_sem_beep =xSemaphoreCreateCounting(255,0);	
	/* 创建事件标志组 */
	g_event_group=xEventGroupCreate();

	xTaskCreate((TaskFunction_t )app_task_beep,             /* 蜂鸣器任务入口函数 */
				(const char*    )"beep",			/* 任务名字 */
				(uint16_t       )128,						/* 任务栈大小 */		
				(void*          )NULL,						/* 任务入口函数参数 */		
				(UBaseType_t    )6,							/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_beep_handle);	/* 任务控制块指针 */ 

	xTaskCreate((TaskFunction_t )app_task_rtc,  			/* RTC任务入口函数 */
				(const char*    )"rtc",				/* 任务名字 */
				(uint16_t       )128,  						/* 任务栈大小 */
				(void*          )NULL,						/* 任务入口函数参数 */
				(UBaseType_t    )6, 							/* 任务的优先级 */
				(TaskHandle_t*  )&app_task_rtc_handle);		/* 任务控制块指针 */

	xTaskCreate((TaskFunction_t )app_task_dht11,  			/* DHT11任务入口函数 */
				(const char*    )"dht11",				/* 任务名字 */
				(uint16_t       )128,  						/* 任务栈大小 */
				(void*          )NULL,						/* 任务入口函数参数 */
				(UBaseType_t    )7, 							/* 任务的优先级 */
				(TaskHandle_t*  )&app_task_dht11_handle);		/* 任务控制块指针 */

	xTaskCreate((TaskFunction_t )app_task_sg90,             /* 舵机任务入口函数 */
				(const char*    )"sg90",			/* 任务名字 */
				(uint16_t       )128,						/* 任务栈大小 */		
				(void*          )NULL,						/* 任务入口函数参数 */		
				(UBaseType_t    )6,							/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_sg90_handle);	/* 任务控制块指针 */ 	

	xTaskCreate((TaskFunction_t )app_task_oled,             /* OLED屏幕任务入口函数 */
				(const char*    )"oled",			/* 任务名字 */
				(uint16_t       )512,						/* 任务栈大小 */		
				(void*          )NULL,						/* 任务入口函数参数 */		
				(UBaseType_t    )7,							/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_oled_handle);	/* 任务控制块指针 */ 

	xTaskCreate((TaskFunction_t )app_task_hcsr04,           /* 超声波任务入口函数 */
				(const char*    )"hcsr04",			/* 任务名字 */
				(uint16_t       )128,						/* 任务栈大小 */		
				(void*          )NULL,						/* 任务入口函数参数 */		
				(UBaseType_t    )7,							/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_hcsr04_handle);	/* 任务控制块指针 */ 	

	xTaskCreate((TaskFunction_t )app_task_bluetooth,           /* 蓝牙任务入口函数 */
				(const char*    )"bluetooth",			/* 任务名字 */
				(uint16_t       )128,						/* 任务栈大小 */		
				(void*          )NULL,						/* 任务入口函数参数 */		
				(UBaseType_t    )6,							/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_bluetooth_handle);	/* 任务控制块指针 */ 

	xTaskCreate((TaskFunction_t )app_task_l298n_wheel,           /* L298N(轮子)任务入口函数 */
				(const char*    )"l298n_wheel",			/* 任务名字 */
				(uint16_t       )512,						/* 任务栈大小 */		
				(void*          )NULL,						/* 任务入口函数参数 */		
				(UBaseType_t    )7,							/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_l298n_wheel_handle);	/* 任务控制块指针 */ 

	xTaskCreate((TaskFunction_t )app_task_mpu6050,           /* mpu6050任务入口函数 */
				(const char*    )"mpu6050",			/* 任务名字 */
				(uint16_t       )512,						/* 任务栈大小 */		
				(void*          )NULL,						/* 任务入口函数参数 */		
				(UBaseType_t    )6,							/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_mpu6050_handle);	/* 任务控制块指针 */ 
		  
	xTaskCreate((TaskFunction_t )app_task_mfrc522,           /* MFRX522任务入口函数 */
				(const char*    )"mfrc522",			/* 任务名字 */
				(uint16_t       )512,						/* 任务栈大小 */		
				(void*          )NULL,						/* 任务入口函数参数 */		
				(UBaseType_t    )6,							/* 任务的优先级 */					
				(TaskHandle_t*  )&app_task_mfrc522_handle);	/* 任务控制块指针 */ 

	//xTaskCreate((TaskFunction_t )app_task_cpu_status,  		/* CPU使用率任务入口函数 */
	//		  (const char*    )"cpu_status",			/* 任务名字 */
	//		  (uint16_t       )512,  				/* 任务栈大小 */
	//		  (void*          )NULL,				/* 任务入口函数参数 */
	//		  (UBaseType_t    )6, 					/* 任务的优先级 */
	//		  (TaskHandle_t*  )&app_task_cpu_status_handle);	/* 任务控制块指针 */ 

	/* 创建周期软件定时器 */
	soft_timer_Handle=xTimerCreate(	(const char*		)"AutoReloadTimer",
									(TickType_t			)1000,/* 定时器周期 1000(tick) */
									(UBaseType_t		)pdTRUE,/* 周期模式 */
									(void*				)1,/* 为每个计时器分配一个索引的唯一ID */
									(TimerCallbackFunction_t)soft_timer_callback); 	
	/* 开启周期软件定时器 */							
	xTimerStart(soft_timer_Handle,0);

	l298n_wheel_config(30);  		//L298N初始化(连接四个电机),速度为20%(满速100)

	/* 初始化OLED */      
	OLED_Init();
	OLED_Clear();
	/* 显示logo */
	OLED_DrawBMP(0,0,128,8,(uint8_t *)pic_logo);									
	/* 持续2秒 */
	vTaskDelay(2000);
	OLED_Clear();

	/* 显示“智能小车” */
	OLED_ShowCHinese(32,0,0);
	OLED_ShowCHinese(48,0,1);
	OLED_ShowCHinese(64,0,2);	
	OLED_ShowCHinese(80,0,3);		

	/*  显示“GZ2132" */
	OLED_ShowString(16,2,(u8 *)"GZ2422",16);

	/* 显示“等待中” */
	OLED_ShowCHinese(64,2,4);
	OLED_ShowCHinese(80,2,5);
	OLED_ShowCHinese(96,2,6);	

	/*  显示"2021/8/18" */
	OLED_ShowString(32,4,(u8 *)"2024/8/18",16);	
	vTaskDelay(3000);
	OLED_Clear();

	/* 放前面防止初始化阶段电机电平飘忽不定 */
	bluetooth_config(115200);    //蓝牙初始化
    /*  查询当前复位是否是看门狗所致  */
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST)==SET)
	{
		usart3_send_str((u8 *)"iwdg reset cpu\r\n");
	}
	else
	{
		usart3_send_str((u8 *)"normal reset cpu\r\n");
	}RCC_ClearFlag();
	LED_Init();
	beep_config();  		//蜂鸣器初始化
	sg90_config(); 			//舵机初始化
	hcsr04_config();  	    //超声波初始化
	dht11_config();         //温湿度初始化

	MPU_Init();
	mpu_dmp_init();

	rtc_config();			//RTC时钟初始化
	MFRC522_Initializtion();  //MFRC522初始化
	/* 独立看门狗初始化 */		
	iwdg_init();        //超时时间2s
	
	vTaskDelete(NULL);		
}

static void app_task_mfrc522(void* pvParameters)   	//任务	mfrc522
{
	uint8_t  	card_pydebuf[2]={0};
	uint8_t  	card_numberbuf[5]={0};  //卡ID
	oled_t		oled;		
	BaseType_t 	xReturn=pdFALSE;
	uint8_t 	ID_buff[5]={0};
	uint8_t     beep_sta=0;

	while(1)
	{
		MFRC522_Initializtion();
		if( 0==MFRC522_Request(0x52, card_pydebuf))			//寻卡
		{
			MFRC522_Anticoll(card_numberbuf);			//防撞处理

		    sprintf((char*)ID_buff,"%02X%02X%02X%02X%02X",card_numberbuf[0],card_numberbuf[1],card_numberbuf[2],card_numberbuf[3],card_numberbuf[4]);
			dgb_printf_safe("%s\r\n",ID_buff);	//调试

			if(strstr((char*)ID_buff,"46256BAFA7"))   //闭锁
			{
				/*  挂起任务，只留下mfrc522任务以及OLED任务  */
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
				xReturn = xQueueSend( 	g_queue_oled,/* 消息队列的句柄 */
										&oled,		/* 发送的消息内容 */
										100);		/* 等待时间 100 Tick */
				if(xReturn != pdPASS)
					dgb_printf_safe("[app_task_key] xQueueSend oled picture error code is %d\r\n",xReturn);

				//蜂鸣器滴一声
				beep_sta=2;
				xReturn = xQueueSend( g_queue_beep,/* 消息队列的句柄 */
									  &beep_sta,	/* 发送的消息内容 */
									  0);		/* 等待时间 100 Tick */			
				if(xReturn != pdPASS)
				{
					dgb_printf_safe("[app_task_mpu6050] xQueueSend beep_sta error code is %d\r\n",xReturn);
					continue;
				}
				/* 阻塞等待信号量，用于确保任务完成对蜂鸣器的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);	
			}
			else if(strstr((char*)ID_buff,"C9F3034D74"))  //解锁
			{
				OLED_Clear();	
				//蜂鸣器滴一声
				beep_sta=2;
				xReturn = xQueueSend( g_queue_beep,/* 消息队列的句柄 */
									  &beep_sta,	/* 发送的消息内容 */
									  0);		/* 等待时间 100 Tick */			
				if(xReturn != pdPASS)
				{
					dgb_printf_safe("[app_task_mpu6050] xQueueSend beep_sta error code is %d\r\n",xReturn);
					continue;
				}
				/* 阻塞等待信号量，用于确保任务完成对蜂鸣器的控制 */
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

static void app_task_rtc(void* pvParameters)  //任务 RTC
{
	uint8_t 		buf[16]={0};	
	oled_t			oled;
	BaseType_t		xReturn;
	EventBits_t 	EventValue;
	RTC_TimeTypeDef RTC_TimeStructure;

	dgb_printf_safe("[app_task_rtc] create success\r\n");		
	
	for(;;)
	{
		/* 等待事件组中的相应事件位，或同步 */
		EventValue=xEventGroupWaitBits((EventGroupHandle_t	)g_event_group,		
									   (EventBits_t			)EVENT_GROUP_RTC_WAKEUP,
									   (BaseType_t			)pdTRUE,				
									   (BaseType_t			)pdFALSE,
									   (TickType_t			)portMAX_DELAY);
		if(EventValue & EVENT_GROUP_RTC_WAKEUP)
		{
			/* RTC_GetTime，获取时间 */
			RTC_GetTime(RTC_Format_BCD, &RTC_TimeStructure); 
				
			/* 格式化字符串 */
			sprintf((char *)buf,"Run: %02x:%02x:%02x",RTC_TimeStructure.RTC_Hours,RTC_TimeStructure.RTC_Minutes,RTC_TimeStructure.RTC_Seconds);
			
			/* oled显示时间 */
			oled.ctrl=OLED_CTRL_SHOW_STRING;
			oled.x=8;
			oled.y=0;
			oled.str=buf;
			oled.font_size=16;

			xReturn = xQueueSend( 	g_queue_oled,/* 消息队列的句柄 */
									&oled,	/* 发送的消息内容 */
									100);		/* 等待时间 100 Tick */
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

			xReturn = xQueueSend( 	g_queue_oled,/* 消息队列的句柄 */
									&oled,		/* 发送的消息内容 */
									100);		/* 等待时间 100 Tick */
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

static void app_task_oled(void *pvParameters)    		 //任务  oled屏幕
{
	oled_t oled;
	BaseType_t xReturn=pdFALSE;	
	dgb_printf_safe("app_task_oled success\r\n");
	while(1)
	{
		xReturn = xQueueReceive( g_queue_oled,	/* 消息队列的句柄 */
								 &oled, 			/* 得到的消息内容 */
								 portMAX_DELAY);	/* 等待时间一直等 */
		if(xReturn != pdPASS)
			continue;

		switch(oled.ctrl)
		{
			case OLED_CTRL_SHOW_STRING:
			{
				/* 显示字符串 */
				OLED_ShowString(oled.x,
								oled.y,
								oled.str,
								oled.font_size);
			}break;

			case OLED_CTRL_SHOW_PICTURE:
			{
				/* 显示图片 */
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

static void app_task_beep(void *pvParameters)    //蜂鸣器任务
{
	uint8_t 	beep_sta=0;	
	BaseType_t	xReturn	=pdFALSE;	
	dgb_printf_safe("app_task_beep success\r\n");
		
	for(;;)
	{
		xReturn = xQueueReceive( g_queue_beep,	/* 消息队列的句柄 */
								 &beep_sta, 		/* 得到的消息内容 */
								portMAX_DELAY);	/* 等待时间一直等 */
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

		/* 释放信号量，告诉对方，当前beep控制任务已经完成 */
		xSemaphoreGive(g_sem_beep);	
		delay_ms(100);	
	}
}


static void app_task_mpu6050(void *pvParameters)       //任务mpu6050
{
	
	uint8_t 	buf[32]={0};	
	float       pitch,roll,yaw; 		//欧拉角
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
			xReturn = xQueueSend( 	g_queue_oled,/* 消息队列的句柄 */
									&oled,	/* 发送的消息内容 */
									0);		/* 等待时间 100 Tick */
			if(xReturn != pdPASS)
				dgb_printf_safe("[app_task_mpu6050] xQueueSend oled string error code is %d\r\n",xReturn);

			
			if(roll<=-60 || roll>=60 ||pitch<=-60 || pitch>=60)
			{
				//BEEP(1);
				beep_sta=1;
				xReturn = xQueueSend( g_queue_beep,/* 消息队列的句柄 */
									  &beep_sta,	/* 发送的消息内容 */
									  0);		/* 等待时间 100 Tick */			
			    if(xReturn != pdPASS)
				{
					dgb_printf_safe("[app_task_mpu6050] xQueueSend beep_sta error code is %d\r\n",xReturn);
					continue;
				}
				/* 阻塞等待信号量，用于确保任务完成对蜂鸣器的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);		
			}else
			{
				//BEEP(0);
				beep_sta=0;
				xReturn = xQueueSend( g_queue_beep,/* 消息队列的句柄 */
									  &beep_sta,	/* 发送的消息内容 */
									  0);		/* 等待时间 100 Tick */			
				if(xReturn != pdPASS)
				{
					dgb_printf_safe("[app_task_mpu6050] xQueueSend beep_sta error code is %d\r\n",xReturn);
					continue;
				}
				/* 阻塞等待信号量，用于确保任务完成对蜂鸣器的控制 */
				xReturn=xSemaphoreTake(g_sem_beep,portMAX_DELAY);					
			}
		}	
		delay_ms(100);
	}		
}

static void app_task_sg90(void *pvParameters)   //舵机任务
{
	uint8_t		 sg90_sta=0;
	BaseType_t 	 xReturn=pdFALSE;	

	dgb_printf_safe("app_task_sg90 success\r\n");

	while(1)
	{
		xReturn = xQueueReceive( g_queue_sg90,	    /* 消息队列的句柄 */
								 &sg90_sta, 		/* 得到的消息内容 */
								 portMAX_DELAY);	/* 等待时间一直等 */
		if(xReturn != pdPASS)
			continue;

		if(sg90_sta==170)  //控制左转      
		{
			sg90_angle(170);  //170° 
			delay_ms(1000);  			
		}		
		else if(sg90_sta==0)  //控制右转      
		{
			sg90_angle(0);  //0° 
			delay_ms(1000);  
		}
		else if(sg90_sta==90)  //控制正面   ————
		{
			sg90_angle(90);  //90° 
			delay_ms(1000);  
		}	
	
		xSemaphoreGive(g_sem_sg90);  /* 释放信号量，告诉对方，当前sg90控制任务已经完成 */

		/*   0°的姿态：|     */

		//TIM_SetCompare3(TIM2,185);//90°   ---   
		//sg90_angle(90);	 //90°   --- 	
		//delay_ms(1000); 		
		
		//TIM_SetCompare3(TIM2,180);//135° 	////
		//sg90_angle(170);  //135° 	////
		//delay_ms(1000); 
		
		//sg90_angle(135);  //135° 	////
		//delay_ms(1000); 

		//TIM_SetCompare3(TIM2,185);//90°   ---  
		//sg90_angle(90);	//90°   ---  	 
		//delay_ms(1000); 

		//TIM_SetCompare3(TIM2,190);//45°   \\\\  	
		//sg90_angle(0);  //45°   \\\\ 
		//delay_ms(1000); 
	}
}


static void app_task_l298n_wheel(void *pvParameters)   //任务  L298N(轮子)
{	
	uint8_t 	sg90_sta=0;
	uint8_t 	hcsr04_sta=0;
	uint32_t    left_distance=0;
	uint32_t    right_distance=0;

	//BaseType_t 	xReturn=pdFALSE;

	dgb_printf_safe("app_task_l298n_wheel success\r\n");

	while(1)
	{
		if(g_wheel_mode == AVOID_MODE)     //避障模式
		{

			/*  控制舵机回到默认位置正面  */
			//sg90_sta=90;   //回到正面90°
			//xReturn = xQueueSend( g_queue_sg90,/* 消息队列的句柄 */
			//						&sg90_sta,	/* 发送的消息内容 */
			//						0);		/* 等待时间 100 Tick */
			///* 阻塞等待信号量，用于确保任务完成对舵机的控制 */
			//xReturn=xSemaphoreTake(g_sem_sg90,portMAX_DELAY);

			//启动超声波工作获取最新数据
			hcsr04_sta=1;
			xQueueSend( g_queue_hcsr04,/* 消息队列的句柄 */
								  &hcsr04_sta,	/* 发送的消息内容 */
								  0);		/* 等待时间 100 Tick */	
			/* 阻塞等待信号量，用于确保任务完成对超声波检测距离获取最新数据 */
			xSemaphoreTake(g_sem_hcsr04,portMAX_DELAY);

			if(g_hcsr04_distance<250)  //距离太近
			{
				/*    后退操作    */
				LEFT_IN1 = 1;      
				LEFT_IN2 = 0;		
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;	
				//dgb_printf_safe("后退");
				delay_ms(900);
				/*    刹车操作    */
				LEFT_IN1=1;      
				LEFT_IN2=1;
				RIGHT_IN1=1;
				RIGHT_IN2=1;	
				//dgb_printf_safe("刹车");

				/*  控制舵机左转  */
				sg90_sta=170;   //左转至170°(左转到180°舵机会卡几下)
				xQueueSend( g_queue_sg90,/* 消息队列的句柄 */
									  &sg90_sta,	/* 发送的消息内容 */
									  0);		/* 等待时间 100 Tick */
				/* 阻塞等待信号量，用于确保任务完成对舵机的控制 */
				xSemaphoreTake(g_sem_sg90,portMAX_DELAY);
				//dgb_printf_safe("舵机左转成功\r\n");

				//启动超声波工作获取左转后的最新数据
				hcsr04_sta=1;
				xQueueSend( g_queue_hcsr04,/* 消息队列的句柄 */
									&hcsr04_sta,	/* 发送的消息内容 */
									0);		/* 等待时间 100 Tick */	
				/* 阻塞等待信号量，用于确保任务完成对超声波检测距离获取最新数据 */
				xSemaphoreTake(g_sem_hcsr04,portMAX_DELAY);
				left_distance = g_hcsr04_distance; //将探测到左边距离存到left变量
				//dgb_printf_safe("超声波测距成功\r\n");//调试超声波成功

				/*  控制舵机右转  */
				sg90_sta=0;   //右转至0°
				xQueueSend( g_queue_sg90,/* 消息队列的句柄 */
									  &sg90_sta,	/* 发送的消息内容 */
									  0);		/* 等待时间 100 Tick */
				/* 阻塞等待信号量，用于确保任务完成对舵机的控制 */
				xSemaphoreTake(g_sem_sg90,portMAX_DELAY);
				//dgb_printf_safe("舵机右转成功\r\n");//调试

				delay_ms(70);   //超声波间隔70ms测,防止来回信号干扰
				//启动超声波工作获取右转后的最新数据
				hcsr04_sta=1;
				xQueueSend( g_queue_hcsr04,/* 消息队列的句柄 */
									&hcsr04_sta,	/* 发送的消息内容 */
									0);		/* 等待时间 100 Tick */	
				/* 阻塞等待信号量，用于确保任务完成对超声波检测距离获取最新数据 */
				xSemaphoreTake(g_sem_hcsr04,portMAX_DELAY);
				right_distance = g_hcsr04_distance; //将探测到右边距离存到right变量
				//dgb_printf_safe("超声波测距成功\r\n");//调试超声波成功
				
				/*  控制舵机回到默认位置正面  */
				sg90_sta=90;   //回到正面90°
				xQueueSend( g_queue_sg90,/* 消息队列的句柄 */
									  &sg90_sta,	/* 发送的消息内容 */
									  0);		/* 等待时间 100 Tick */
				/* 阻塞等待信号量，用于确保任务完成对舵机的控制 */
				xSemaphoreTake(g_sem_sg90,portMAX_DELAY);

				/*  左侧距离大于右边   */
				if(left_distance>right_distance)  
				{
					/*  左转操作  */
					LEFT_IN1=1;      //左电机后退;
					LEFT_IN2=0;
					RIGHT_IN1=0;     //右电机前进
					RIGHT_IN2=1;
					//dgb_printf_safe("左转");

					delay_ms(500);  //延时550ms后下次回到安全距离实行前进操作即可实行90°左转
				}
				else    /*  右侧距离大于左边   */
				{
					/*  右转操作  */
					LEFT_IN1 = 0;      //左电机前进;
					LEFT_IN2 = 1;
					RIGHT_IN1 = 1;     //右电机后退
					RIGHT_IN2 = 0;
					//dgb_printf_safe("右转");

					delay_ms(500); //延时550ms后下次回到安全距离实行前进操作即可实行90°右转
				}
			}
			else      //距离安全
			{
				/*  前进操作  */
				LEFT_IN1 =0;    //左电机前进
				LEFT_IN2 =1;
				RIGHT_IN1=0;    //右电机前进
				RIGHT_IN2=1;
				//dgb_printf_safe("前进\r\n");				
			}
		}
		else if(g_wheel_mode == FOLLOW_MODE)      //跟随模式
		{
			hcsr04_sta=1;
			xQueueSend( g_queue_hcsr04,/* 消息队列的句柄 */
								  &hcsr04_sta,	/* 发送的消息内容 */
								  0);		/* 等待时间 100 Tick */	
			/* 阻塞等待信号量，用于确保任务完成对超声波检测距离获取最新数据 */
			xSemaphoreTake(g_sem_hcsr04,portMAX_DELAY);	
			//dgb_printf_safe("超声波测距成功\r\n"); //调式超声波测距成功;

			if(g_hcsr04_distance>1000)      //超出100cm外没有人(则停止)
			{
				/*  刹车操作  */
				LEFT_IN1=1;     
				LEFT_IN2=1;
				RIGHT_IN1=1;
				RIGHT_IN2=1;
				//dgb_printf_safe("刹车\r\n");	
			}
			else if(g_hcsr04_distance>=400&&g_hcsr04_distance<1000)  //在40cm-100cm范围有人(则前进)
			{				
				/*  前进操作  */
				LEFT_IN1 =0;    
				LEFT_IN2 =1;
				RIGHT_IN1=0;    
				RIGHT_IN2=1;	
				//dgb_printf_safe("前进\r\n");		
			}
			else if(g_hcsr04_distance<400 && g_hcsr04_distance>=200)   //20cm-40cm范围有人(则刹车)
			{
				/*  刹车操作  */
				LEFT_IN1=1;      
				LEFT_IN2=1;
				RIGHT_IN1=1;
				RIGHT_IN2=1;
				//dgb_printf_safe("刹车\r\n");	
			}
			else if(g_hcsr04_distance<200)    //在20cm内有人(则后退直到安全距离20cm-40cm范围内方可刹车)
			{
				/*  后退操作  */
				LEFT_IN1 = 1;      
				LEFT_IN2 = 0;		
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;	
				//dgb_printf_safe("后退\r\n");	
			}				
		}
		else if(g_wheel_mode == MANUAL_MODE)    //手动模式
		{
			if(g_wheel_status == WHEEL_STOP)      //刹车
			{
				/*  刹车操作  */
				LEFT_IN1=1;      
				LEFT_IN2=1;
				RIGHT_IN1=1;
				RIGHT_IN2=1;
				//dgb_printf_safe("刹车\r\n");	
			}
			else if(g_wheel_status == WHEEL_UP)   //前进
			{
				/*  前进操作  */
				LEFT_IN1 =0;    
				LEFT_IN2 =1;
				RIGHT_IN1=0;    
				RIGHT_IN2=1;	

				//dgb_printf_safe("前进");

			}
			else if(g_wheel_status == WHEEL_DOWN)  //后退
			{
				/*  后退操作  */
				LEFT_IN1 = 1;      
				LEFT_IN2 = 0;		
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;	
				//dgb_printf_safe("后退\r\n");	
			}
			else if(g_wheel_status == WHEEL_LEFT)  //90°左转
			{
				/*  左转操作  */
				LEFT_IN1=1;      
				LEFT_IN2=0;
				RIGHT_IN1=0;     
				RIGHT_IN2=1;
				//dgb_printf_safe("左转");

				delay_ms(500);

				g_wheel_status = WHEEL_UP;  //延时550ms后改为前进状态即可实现90°左转
			}
			else if(g_wheel_status == WHEEL_RIGHT)  //90°右转
			{
				/*  右转操作  */
				LEFT_IN1 = 0;      
				LEFT_IN2 = 1;
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;
				//dgb_printf_safe("右转");

				delay_ms(500);
				g_wheel_status = WHEEL_UP;  //延时550ms后改为前进状态即可实现90°右转

			}
			else if(g_wheel_status == WHEEL_ORIGIN_LEFT)  //原地左转
			{
				/*  原地左转操作  */
				LEFT_IN1=1;      
				LEFT_IN2=0;
				RIGHT_IN1=0;     
				RIGHT_IN2=1;
				//dgb_printf_safe("原地左转\r\n");
			}
			else if(g_wheel_status == WHEEL_ORIGIN_RIGHT)  //原地右转 
			{
				/*  原地右转操作  */
				LEFT_IN1 = 0;      
				LEFT_IN2 = 1;
				RIGHT_IN1 = 1;     
				RIGHT_IN2 = 0;
				//dgb_printf_safe("原地右转\r\n");
			}			
		}
		else     //紧急停止模式
		{
			LEFT_IN1=1;      
			LEFT_IN2=1;
			RIGHT_IN1=1;
			RIGHT_IN2=1;
			//dgb_printf_safe("刹车\r\n");

			/*  控制舵机回到默认位置正面  */
			sg90_sta=90;   //回到正面90°
			xQueueSend( g_queue_sg90,/* 消息队列的句柄 */
									&sg90_sta,	/* 发送的消息内容 */
									0);		/* 等待时间 100 Tick */
			/* 阻塞等待信号量，用于确保任务完成对舵机的控制 */
			xSemaphoreTake(g_sem_sg90,portMAX_DELAY);		
		    dgb_printf_safe("舵机回到初始位置\r\n");//调试
		}
		delay_ms(10);
	}
}

static void app_task_hcsr04(void *pvParameters)    //任务  超声波测距
{
	uint8_t hcsr04_sta=0;
	BaseType_t xReturn=pdFALSE;	

	dgb_printf_safe("app_task_hcsr04 success\r\n");
	while(1)
	{
		xReturn = xQueueReceive( g_queue_hcsr04,	    /* 消息队列的句柄 */
								 &hcsr04_sta, 		/* 得到的消息内容 */
								 portMAX_DELAY);	/* 等待时间一直等 */
		if(xReturn != pdPASS)
			continue;
		
		if(hcsr04_sta)
		{
			g_hcsr04_distance=hcsr04_get_distance();
			//测距异常
			//if(g_hcsr04_distance<20 || g_hcsr04_distance>4000)
			//{
			//	continue;
			//}
			//dgb_printf_safe("distance = %dmm\r\n",g_hcsr04_distance);
			xSemaphoreGive(g_sem_hcsr04);  /* 释放信号量，告诉对方，当前sg90控制任务已经完成 */

		}
	}
}

static void app_task_bluetooth(void *pvParameters)    //任务  蓝牙
{
	static  uint8_t  bluetooth_recv_buf[QUEUE_BLUETOOTH_SIZE]={0};

	BaseType_t xReturn=pdFALSE;	

	dgb_printf_safe("app_task_bluetooth success\r\n");

	while(1)
	{
		xReturn = xQueueReceive( g_queue_bluetooth,  /* 消息队列的句柄 */
								 bluetooth_recv_buf, /* 得到的消息内容 */
								 portMAX_DELAY);     /* 等待时间一直等 */		

		if(xReturn != pdPASS)
		{
			dgb_printf_safe("[app_task_bluetooth] xQueueReceive bluetooth_recv_buf error code is %d\r\n",xReturn);
			
			continue;
		}
		dgb_printf_safe("%s\r\n",bluetooth_recv_buf);  //调试	
		if(strstr((char *)bluetooth_recv_buf,"avoid_mode#"))          //避障模式
		{
			/* 恢复超声波任务和舵机任务 */
			//vTaskResume(app_task_hcsr04_handle);
			//vTaskResume(app_task_sg90_handle);	

			g_wheel_mode = AVOID_MODE;
		} 
		else if(strstr((char *)bluetooth_recv_buf,"manual_mode#"))    //手动模式  
		{
			/* 挂起用不到的超声波任务和舵机任务 */
			//vTaskSuspend(app_task_hcsr04_handle);	
			//vTaskSuspend(app_task_sg90_handle);	

			g_wheel_mode = MANUAL_MODE;

		}
		else if(strstr((char *)bluetooth_recv_buf,"follow_mode#"))    //跟随模式
		{
			//vTaskSuspend(app_task_sg90_handle);	/* 挂起用不到的舵机任务 */
			//vTaskResume(app_task_hcsr04_handle);/* 恢复超声波任务 */

			g_wheel_mode = FOLLOW_MODE;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"stop_mode#"))    //紧急停止模式
		{
			//vTaskResume(app_task_sg90_handle);	/* 挂起用不到的舵机任务 */
			//vTaskSuspend(app_task_hcsr04_handle);/* 恢复超声波任务 */

			g_wheel_mode = STOP_MODE;
			
		}		
		else if(strstr((char *)bluetooth_recv_buf,"stop#"))           //刹车
		{

			g_wheel_status = WHEEL_STOP;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"up#"))             //前进
		{

			g_wheel_status = WHEEL_UP;

		}
		else if(strstr((char *)bluetooth_recv_buf,"down#"))           //后退
		{

			g_wheel_status = WHEEL_DOWN;

		}
		/*     原地左转和原地右转要放在左转和右转的后面  */
		else if(strstr((char *)bluetooth_recv_buf,"origin_left#"))			  //原地左转		
		{

			g_wheel_status = WHEEL_ORIGIN_LEFT;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"origin_right#"))          //原地右转
		{

			g_wheel_status = WHEEL_ORIGIN_RIGHT;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"left#"))			  //左转		
		{

			g_wheel_status = WHEEL_LEFT;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"right#"))          //右转
		{

			g_wheel_status = WHEEL_RIGHT;
			
		}
		else if(strstr((char *)bluetooth_recv_buf,"stop#"))           //刹车
		{

			g_wheel_status = WHEEL_STOP;
			
		}

		memset(bluetooth_recv_buf,0,sizeof bluetooth_recv_buf);	  //清除bluetooth_recv_buf内容

	}
}



static void soft_timer_callback(void* parameter)  //1s喂狗一次
{		
	/* 关闭中断 */
	portDISABLE_INTERRUPTS();
	
	/* 喂狗，刷新自身计数值 */
	IWDG_ReloadCounter();	
	
	//dgb_printf_safe("7");
	/* 打开中断 */
	portENABLE_INTERRUPTS();
	
	//TickType_t tick_cnt;
	//tick_cnt = xTaskGetTickCount();	/* 获取滴答定时器的计数值 */
} 

/*************************************************************************
		以下必须要提供的，且在FreeRTOSConfig.h
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()    ConfigureTimeForRunTimeStats()//定时器3提供时间统计的时基，频率为10K，即周期为100us
#define portGET_RUN_TIME_COUNTER_VALUE()			FreeRTOSRunTimeTicks		//获取时间统计时间值	
 *************************************************************************/
#if 0
static void app_task_cpu_status(void *pvParameters)
{
	BaseType_t xReturn=pdFALSE;	
	
	uint8_t cpu_run_info[400];		//保存任务运行时间信息
	
	for(;;)
	{
		memset(cpu_run_info,0,400);				//信息缓冲区清零
		
		vTaskList((char *)&cpu_run_info);  //获取任务运行时间信息
		
		printf("---------------------------------------------\r\n");
		printf("任务名      任务状态 优先级   剩余栈 任务序号\r\n");
		printf("%s", cpu_run_info);
		printf("---------------------------------------------\r\n");
		
		memset(cpu_run_info,0,400);				//信息缓冲区清零
		
		vTaskGetRunTimeStats((char *)&cpu_run_info);
		
		printf("任务名       运行计数         使用率\r\n");
		printf("%s", cpu_run_info);
		printf("---------------------------------------------\r\n\n");
		vTaskDelay(1000);   /* 延时1000个tick */	
	}
} 

//FreeRTOS时间统计所用的节拍计数器
volatile unsigned long long FreeRTOSRunTimeTicks;
//初始化TIM3使其为FreeRTOS的时间统计提供时基
void ConfigureTimeForRunTimeStats(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	FreeRTOSRunTimeTicks=0;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Period   = 100; 			//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler= 84-1;  		//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		FreeRTOSRunTimeTicks++;
	}
	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
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
