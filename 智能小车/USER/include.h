#ifndef __INCLUDE_H
#define __INCLUDE_H

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "limits.h"


#include "stm32f4xx.h" 
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdint.h>

#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dht11.h"
#include "hcsr04.h"
#include "oled.h"
#include "bmp.h"
#include "rtc.h"
#include "sys.h"
#include "sg90.h"
#include "beep.h"
#include "l298n.h"
#include "bluetooth.h"
#include "iwdg.h"
#include "MFRC522.h"

//mpu6050
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"


/* 宏定义 */
#define  WHEEL_UP               0x01        //前进
#define  WHEEL_DOWN             0x02        //后退
#define  WHEEL_LEFT             0x04        //左转
#define  WHEEL_RIGHT            0x08        //右转
#define  WHEEL_STOP             0x10        //刹车
#define  WHEEL_ORIGIN_LEFT      0x20        //原地左转
#define  WHEEL_ORIGIN_RIGHT     0x40        //原地右转

#define  AVOID_MODE      0x100       //避障模式
#define  MANUAL_MODE     0x200       //手机模式
#define  FOLLOW_MODE     0x400       //跟随模式
#define  STOP_MODE       0x800        //紧急停止模式

#define  EVENT_GROUP_RTC_WAKEUP      0x1000

#define  QUEUE_BLUETOOTH_LEN          4        /* 队列的长度，最大可包含多少个消息 */
#define  QUEUE_BLUETOOTH_SIZE         20       /* 队列中每个消息大小（字节） */
#define  QUEUE_BEEP_LEN    		4   	/* 队列的长度，最大可包含多少个消息 */

#define  QUEUE_SG90_LEN    		4   	/* 队列的长度，最大可包含多少个消息 */
#define  QUEUE_HCSR04_LEN       4      /* 队列的长度，最大可包含多少个消息 */
#define QUEUE_OLED_LEN    		16   	/* 队列的长度，最大可包含多少个消息 */

/* 变量 */
extern  QueueHandle_t   g_queue_bluetooth;
extern SemaphoreHandle_t g_mutex_printf;     //USART1
extern EventGroupHandle_t 	g_event_group;	


/* 类型 */
typedef struct __oled_t
{

#define OLED_CTRL_DISPLAY_ON        0x01
#define OLED_CTRL_DISPLAY_OFF       0x02
#define OLED_CTRL_INIT              0x03
#define OLED_CTRL_CLEAR             0x04
#define OLED_CTRL_SHOW_STRING       0x05
#define OLED_CTRL_SHOW_CHINESE      0x06
#define OLED_CTRL_SHOW_PICTURE      0x07


	uint8_t ctrl;
	uint8_t x;
	uint8_t y;

	uint8_t *str;
	uint8_t font_size;
    uint8_t chinese;
	
	const uint8_t *pic;
	uint8_t pic_width;
	uint8_t pic_height;
}oled_t;


#endif


