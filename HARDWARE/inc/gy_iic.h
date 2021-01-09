#ifndef   _GY_IIC_H
#define   _GY_IIC_H

#include "stdint.h"

#define GY_SCL_H         GPIOB->BSRR = GPIO_Pin_5
#define GY_SCL_L         GPIOB->BRR  = GPIO_Pin_5
#define GY_SDA_H         GPIOB->BSRR = GPIO_Pin_6 
#define GY_SDA_L         GPIOB->BRR  = GPIO_Pin_6
#define GY_SDA_read      ((GPIOB->IDR & GPIO_Pin_6)!=0)?1:0 //��ξ���

void GY_SDA_OUT(void);
void GY_SDA_IN(void);

void GY_IIC_GPIO_Init(void);        //��ʼ��IIC��IO��				 
void GY_IIC_Start(void);			 //����IIC��ʼ�ź�
void GY_IIC_Stop(void);	  	  	 //����IICֹͣ�ź�
void GY_IIC_Ack(void);				 //IIC����ACK�ź�
void GY_IIC_NAck(void);			 //IIC������ACK�ź�
uint8_t GY_IIC_WaitAck(void); 		 //IIC�ȴ�ACK�ź�



void GY_IIC_SendByte(uint8_t data);  //IIC����һ���ֽ�
uint8_t GY_IIC_ReadByte(uint8_t ack);//IIC��ȡһ���ֽ�

uint8_t GY_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t GY_IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t GY_IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t GY_IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);

#endif


