#ifndef   _MAIN_H
#define   _MAIN_H

#include "stm32f10x_it.h" 
#include "stdio.h"
#include "stm32f10x.h"
#include "nvic.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "iic.h"
#include "exit.h"
#include "tim.h"
#include "motor.h"
#include "mpu6050.h"
#include "imu.h"
#include "usart.h"
#include "usart2.h"
#include "oled.h"
#include "pid.h"
#include "dat_deal.h"
#include "control.h"
#include "gy_30.h"
#include "gy_iic.h"
#include "myiic.h"
#include "MS5837.h"


void System_Init(void);
void Task_Schedule(void);

extern void delay_init(void);
extern void delay_ms(u16 nms);
extern void delay_us(u32 nus);
#endif

