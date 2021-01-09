/*******************************************************************************************
* LED显示
* 程序版本：V1.0
* 程序日期：2020-8-18
* 程序作者：徐杰
*******************************************************************************************/
#include "stm32f10x.h"
#include "exit.h"
#include "delay.h"
#include "stdio.h"

/****************************************************************************************************
* 函  数: void Exit_Init(void)
* 功  能: 配置与MPU6050的IRQ-PB14
* 参  数: 无
* 返回值：无
* 备  注: 无
****************************************************************************************************/
void Exit_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;   //定义GPIO结构体变量
	EXTI_InitTypeDef EXTI_InitStruct;	//定义外部中断结构体变量
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO,ENABLE);   //使能GPIOB的时钟并开启复用时钟
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);   //PB2中断线映射
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_14;   //配置GPIO第14引脚
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;   //配置GPIO为上拉输入
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;   //配置GPIO速率
	GPIO_Init(GPIOB,&GPIO_InitStruct);   //GPIO初始化函数
	
	
	EXTI_InitStruct.EXTI_Line=EXTI_Line14;   //中断线14
	EXTI_InitStruct.EXTI_Mode=EXTI_Mode_Interrupt;   //外部中断模式
	EXTI_InitStruct.EXTI_Trigger=EXTI_Trigger_Falling;   //下降沿触发
	EXTI_InitStruct.EXTI_LineCmd=ENABLE;   //中断线使能
	EXTI_Init(&EXTI_InitStruct);   //外部中断初始化函数
	
}

