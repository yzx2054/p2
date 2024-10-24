#ifndef __MOTOR_WHEEL_H
#define __MOTOR_WHEEL_H

/*  左边电机控制时钟端口、引脚定义 */
#define LEFT_IN1                    PFout(12)
#define LEFT_IN2                    PDout(4)

/*  右边电机控制时钟端口、引脚定义 */
#define RIGHT_IN1                   PDout(14)
#define RIGHT_IN2                   PDout(0)

void l298n_wheel_config(uint32_t duty);            //L298N初始化(连接四个电机),参数为速度(满速100)

#endif
