#include "include.h"

void usart3_send_str(uint8_t *str)
{
	uint16_t i=0;		

	do
	{
		USART_SendData(USART3,str[i++]);
		while(!USART_GetFlagStatus(USART3,USART_FLAG_TXE));
	}while(str[i]!=0);	
}

void bluetooth_config(u32 baud)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB10与GPIOB11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9，PA10

   //USART3 初始化设置
	USART_InitStructure.USART_BaudRate = baud;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	USART_Cmd(USART3, ENABLE);  //使能串口3 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

	printf("bluetooth_config ok\r\n");

}

//串口1中断服务程序
void USART3_IRQHandler(void)                	
{
	static  uint8_t    rxbuf[20]={0};
	static  uint32_t   i = 0;
	uint8_t  d;
	uint32_t  ulReturn;
	
	/* 进入临界段，临界段可以嵌套 */
	ulReturn = taskENTER_CRITICAL_FROM_ISR();	
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  
	{
		//接收串口数据
		d=USART_ReceiveData(USART3);

		rxbuf[i++]=d;

		//xSemaphoreTakeFromISR(g_mutex_printf,NULL); // 获取互斥信号量 
		//USART_SendData(USART1,d);	
		//xSemaphoreGiveFromISR(g_mutex_printf,NULL);		  // 释放互斥信号量 

		//检测到'#'符或接收的数据满的时候则发送数据
		if(d=='#' || i>=(sizeof rxbuf))
		{
			xQueueSendFromISR(g_queue_bluetooth,(void *)rxbuf,NULL);
			memset(rxbuf,0,sizeof rxbuf);
			i=0;
		}			

		//清空串口3接收中断标志位
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	} 
	
	/* 退出临界段 */
	taskEXIT_CRITICAL_FROM_ISR( ulReturn );	
} 
