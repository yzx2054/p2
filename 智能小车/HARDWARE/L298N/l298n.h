#ifndef __MOTOR_WHEEL_H
#define __MOTOR_WHEEL_H

/*  ��ߵ������ʱ�Ӷ˿ڡ����Ŷ��� */
#define LEFT_IN1                    PFout(12)
#define LEFT_IN2                    PDout(4)

/*  �ұߵ������ʱ�Ӷ˿ڡ����Ŷ��� */
#define RIGHT_IN1                   PDout(14)
#define RIGHT_IN2                   PDout(0)

void l298n_wheel_config(uint32_t duty);            //L298N��ʼ��(�����ĸ����),����Ϊ�ٶ�(����100)

#endif
