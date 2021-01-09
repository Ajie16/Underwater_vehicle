#ifndef   _GY_IIC_H
#define   _GY_IIC_H

#include "stdint.h"

#define GY_SCL_H         GPIOB->BSRR = GPIO_Pin_5
#define GY_SCL_L         GPIOB->BRR  = GPIO_Pin_5
#define GY_SDA_H         GPIOB->BSRR = GPIO_Pin_6 
#define GY_SDA_L         GPIOB->BRR  = GPIO_Pin_6
#define GY_SDA_read      ((GPIOB->IDR & GPIO_Pin_6)!=0)?1:0 //这段绝了

void GY_SDA_OUT(void);
void GY_SDA_IN(void);

void GY_IIC_GPIO_Init(void);        //初始化IIC的IO口				 
void GY_IIC_Start(void);			 //发送IIC开始信号
void GY_IIC_Stop(void);	  	  	 //发送IIC停止信号
void GY_IIC_Ack(void);				 //IIC发送ACK信号
void GY_IIC_NAck(void);			 //IIC不发送ACK信号
uint8_t GY_IIC_WaitAck(void); 		 //IIC等待ACK信号



void GY_IIC_SendByte(uint8_t data);  //IIC发送一个字节
uint8_t GY_IIC_ReadByte(uint8_t ack);//IIC读取一个字节

uint8_t GY_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t GY_IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t GY_IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t GY_IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);

#endif


