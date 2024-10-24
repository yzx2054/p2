#ifndef __BEEP_H
#define __BEEP_H

/*  蜂鸣器时钟端口、引脚定义 */
#define BEEP_PORT 			GPIOF   
#define BEEP_PIN 			GPIO_Pin_8
#define BEEP_PORT_RCC		RCC_AHB1Periph_GPIOF
#define BEEP(x)		PFout(8)=(x)


void beep_config(void);

#endif
