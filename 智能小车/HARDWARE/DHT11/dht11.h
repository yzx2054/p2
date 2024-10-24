#ifndef __DHT11_H
#define __DHT11_H

void dht11_config(void);
int8_t dht11_read(uint8_t *buff);  //起始信号     //0：没检测到dht11   1：检测到dht11



#endif
