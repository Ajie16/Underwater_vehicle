#include "gy_iic.h"
#include "stm32f10x.h"
#include "delay.h"


void GY_SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void GY_SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void GY_IIC_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;   
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);   
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_5|GPIO_Pin_6; 
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP; 
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_5|GPIO_Pin_6);
}	
void GY_IIC_Start(void)
{
	GY_SDA_OUT(); //SDA����� 
	GY_SDA_H;
	GY_SCL_H;	
	delay_us(4);
 	GY_SDA_L;
	delay_us(4);
	GY_SCL_L;
}

void GY_IIC_Stop(void)
{
	GY_SDA_OUT(); //SDA�����
	GY_SCL_L;
	GY_SDA_L;
	delay_us(4);
	GY_SCL_H; 
	GY_SDA_H;
	delay_us(4);	
}
void GY_IIC_Ack(void)
{
	GY_SCL_L;
	GY_SDA_OUT();
	GY_SDA_L;
	delay_us(1);
	GY_SCL_H;
	delay_us(2);
	GY_SCL_L;
}
void GY_IIC_NAck(void)
{
	GY_SCL_L;
	GY_SDA_OUT();
	GY_SDA_H;
	delay_us(1);
	GY_SCL_H;
	delay_us(1);
	GY_SCL_L;
}
uint8_t GY_IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	GY_SDA_IN(); //SDA����Ϊ����  ���ӻ���һ���͵�ƽ��ΪӦ�� 
	GY_SDA_H;
	delay_us(1);	   
	GY_SCL_H;
	delay_us(1);;	 
	while(GY_SDA_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			GY_IIC_Stop();
			return 1;
		}
	}
	GY_SCL_L; //ʱ�������	   
	return 0; 
}

void GY_IIC_SendByte(uint8_t data)
{
	uint8_t t;   
	GY_SDA_OUT(); 	    
	GY_SCL_L; //����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{
	if(data&0x80)
		GY_SDA_H;
	else
		GY_SDA_L;
	data <<= 1;
	delay_us(1);			
	GY_SCL_H;
	delay_us(1);
	GY_SCL_L;	
	delay_us(1);
	}	 
}
uint8_t GY_IIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
	GY_SDA_IN(); //SDA����Ϊ����ģʽ �ȴ����մӻ���������
	for(i=0;i<8;i++ )
	{
		GY_SCL_L; 
		delay_us(1);
		GY_SCL_H;
		receive<<=1;
		if(GY_SDA_read)receive++; //�ӻ����͵ĵ�ƽ
			delay_us(1); 
	}					 
    if(ack)
        GY_IIC_Ack(); //����ACK 
    else
        GY_IIC_NAck(); //����nACK  
    return receive;
}



//-----------IIC���ݽ���----------------//


/******************************************************************************
* ��  ����uint8_t GY_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
* �����ܣ���ȡָ���豸 ָ���Ĵ�����һ��ֵ
* ��  ����I2C_Addr  Ŀ���豸��ַ
		  reg	    �Ĵ�����ַ
          *buf      ��ȡ����Ҫ�洢�ĵ�ַ    
* ����ֵ������ 1ʧ�� 0�ɹ�
* ��  ע����
*******************************************************************************/ 
uint8_t GY_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	GY_IIC_Start();	
	GY_IIC_SendByte(I2C_Addr);	 //���ʹӻ���ַ
	if(GY_IIC_WaitAck()) //����ӻ�δӦ�������ݷ���ʧ��
	{
		GY_IIC_Stop();
		return 1;
	}
	GY_IIC_SendByte(reg); //���ͼĴ�����ַ
	GY_IIC_WaitAck();	  
	
	GY_IIC_Start();
	GY_IIC_SendByte(I2C_Addr+1); //�������ģʽ			   
	GY_IIC_WaitAck();
	*buf=GY_IIC_ReadByte(0);	   
	GY_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/*************************************************************************************
* ��  ����uint8_t GY_IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr��uint8_t buf))
* �����ܣ�д��ָ���豸 ָ���Ĵ�����һ��ֵ
* ��  ����I2C_Addr  Ŀ���豸��ַ
		  reg	    �Ĵ�����ַ
          buf       Ҫд�������
* ����ֵ��1 ʧ�� 0�ɹ�
* ��  ע����
**************************************************************************************/ 
uint8_t GY_IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	GY_IIC_Start();
	GY_IIC_SendByte(I2C_Addr); //���ʹӻ���ַ
	if(GY_IIC_WaitAck())
	{
		GY_IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	GY_IIC_SendByte(reg); //���ͼĴ�����ַ
	GY_IIC_WaitAck();	  
	GY_IIC_SendByte(data); 
	if(GY_IIC_WaitAck())
	{
		GY_IIC_Stop(); 
		return 1; //����д��ʧ��
	}
	GY_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/***************************************************************************************
* ��  ����uint8_t GY_IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
* �����ܣ���ȡָ���豸 ָ���Ĵ����� length��ֵ
* ��  ����dev     Ŀ���豸��ַ
		  reg	  �Ĵ�����ַ
          length  Ҫ�����ֽ���
		  *data   ���������ݽ�Ҫ��ŵ�ָ��
* ����ֵ��1�ɹ� 0ʧ��
* ��  ע����
***************************************************************************************/ 
uint8_t GY_IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;
	uint8_t temp;
	GY_IIC_Start();
	GY_IIC_SendByte(dev); //���ʹӻ���ַ
	if(GY_IIC_WaitAck())
	{
		GY_IIC_Stop(); 
		return 1; //�ӻ���ַд��ʧ��
	}
	GY_IIC_SendByte(reg); //���ͼĴ�����ַ
	GY_IIC_WaitAck();	  
	GY_IIC_Start();
	GY_IIC_SendByte(dev+1); //�������ģʽ	
	GY_IIC_WaitAck();
	for(count=0;count<length;count++)
	{
		if(count!=(length-1))
			temp = GY_IIC_ReadByte(1); //��ACK�Ķ�����
		else  
			temp = GY_IIC_ReadByte(0); //���һ���ֽ�NACK

		data[count] = temp;
	}
  GY_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}

/****************************************************************************************
* ��  ����uint8_t GY_IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
* �����ܣ�������ֽ�д��ָ���豸 ָ���Ĵ���
* ��  ����dev     Ŀ���豸��ַ
		  reg	  �Ĵ�����ַ
		  length  Ҫд���ֽ���
		  *data   Ҫд������ݽ�Ҫ��ŵ�ָ��
* ����ֵ��1�ɹ� 0ʧ��
* ��  ע����
****************************************************************************************/ 
uint8_t GY_IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
 	uint8_t count = 0;
	GY_IIC_Start();
	GY_IIC_SendByte(dev); //���ʹӻ���ַ
	if(GY_IIC_WaitAck())
	{
		GY_IIC_Stop();
		return 1; //�ӻ���ַд��ʧ��
	}
	GY_IIC_SendByte(reg); //���ͼĴ�����ַ
	GY_IIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		GY_IIC_SendByte(data[count]); 
		if(GY_IIC_WaitAck()) //ÿһ���ֽڶ�Ҫ�ȴӻ�Ӧ��
		{
			GY_IIC_Stop();
			return 1; //����д��ʧ��
		}
	}
	GY_IIC_Stop(); //����һ��ֹͣ����
	return 0;
}





