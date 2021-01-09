#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
/////////////////////////////////////////////////////////////////////////////////////////		 
//B30 ��ȴ�������������
//�����壺BlueTest STM32
//������̳: www.Bluerobots.cn ��BlueRobots ˮ�»�����������
//�޸�����: 2019/4/30
//���̰汾��V1.2
//��ϵ���䣺info@bluerobots.cn
//�ر���������������Դ�����磬��BlueRobots ���������޸ĺ����ڽ�������ʹ�������ге�һ�к����
/////////////////////////////////////////////////////////////////////////////////////////	
   	   		   
////IO��������, 
//#define SDA_IN()  {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=8<<12;} // PC11 = SDA ��Ϊ����ʱ 
//#define SDA_OUT() {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=3<<12;} // PC11 = SDA ��Ϊ���ʱ

////IO��������	 
//#define IIC_SCL    PCout(10) //SCL = PC10
//#define IIC_SDA    PCout(11) //SDA = PC11 ��Ϊ���ʱ
//#define READ_SDA   PCin(11)  //SDA = PC11 ��Ϊ����ʱ

#define GY_SDA_IN()  {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=8<<16;}
#define GY_SDA_OUT() {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=3<<16;}

#define GY_IIC_SCL    PBout(3) //SCL
#define GY_IIC_SDA    PBout(4) //SDA	 
#define GY_READ_SDA   PBin(4)  //����SDA 
 
//IIC���в�������
void GY_IIC_Init(void);                //��ʼ��IIC��IO��				 
void GY_IIC_Start(void);								//����IIC��ʼ�ź�
void GY_IIC_Stop(void);	  						//����IICֹͣ�ź�
void GY_IIC_Send_Byte(u8 txd);					//IIC����һ���ֽ�
u8 GY_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 GY_IIC_Wait_Ack(void); 							//IIC�ȴ�ACK�ź�
void GY_IIC_Ack(void);									//IIC����ACK�ź�
void GY_IIC_NAck(void);								//IIC������ACK�ź�

void GY_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 GY_IIC_Read_One_Byte(u8 daddr,u8 addr);	  

#endif


















