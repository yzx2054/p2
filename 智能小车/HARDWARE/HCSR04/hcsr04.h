#ifndef __HCSR04_H
#define __HCSR04_H

/*  超声波时钟端口、引脚定义 */
/*     Triq触发信号引脚         */
#define HCSR_TRIQ_PORT 			GPIOE   
#define HCSR_TRIQ_PIN 			GPIO_Pin_7
#define HCSR_TRIQ_PORT_RCC		RCC_AHB1Periph_GPIOE
#define TRIQ_W                  PEout(7)
/*     Echo回响信号引脚         */
#define HCSR_ECHO_PORT 			GPIOE   
#define HCSR_ECHO_PIN 			GPIO_Pin_9
#define HCSR_ECHO_PORT_RCC		RCC_AHB1Periph_GPIOE
#define ECHO_R                  PEin(9)




void hcsr04_config(void);
int32_t hcsr04_get_distance(void);   //

#endif
