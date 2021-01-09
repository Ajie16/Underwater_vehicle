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
	GY_SDA_OUT(); //SDA线输出 
	GY_SDA_H;
	GY_SCL_H;	
	delay_us(4);
 	GY_SDA_L;
	delay_us(4);
	GY_SCL_L;
}

void GY_IIC_Stop(void)
{
	GY_SDA_OUT(); //SDA线输出
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
	GY_SDA_IN(); //SDA设置为输入  （从机给一个低电平做为应答） 
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
	GY_SCL_L; //时钟输出低	   
	return 0; 
}

void GY_IIC_SendByte(uint8_t data)
{
	uint8_t t;   
	GY_SDA_OUT(); 	    
	GY_SCL_L; //拉低时钟开始数据传输
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
	GY_SDA_IN(); //SDA设置为输入模式 等待接收从机返回数据
	for(i=0;i<8;i++ )
	{
		GY_SCL_L; 
		delay_us(1);
		GY_SCL_H;
		receive<<=1;
		if(GY_SDA_read)receive++; //从机发送的电平
			delay_us(1); 
	}					 
    if(ack)
        GY_IIC_Ack(); //发送ACK 
    else
        GY_IIC_NAck(); //发送nACK  
    return receive;
}



//-----------IIC数据交互----------------//


/******************************************************************************
* 函  数：uint8_t GY_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
* 功　能：读取指定设备 指定寄存器的一个值
* 参  数：I2C_Addr  目标设备地址
		  reg	    寄存器地址
          *buf      读取数据要存储的地址    
* 返回值：返回 1失败 0成功
* 备  注：无
*******************************************************************************/ 
uint8_t GY_IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	GY_IIC_Start();	
	GY_IIC_SendByte(I2C_Addr);	 //发送从机地址
	if(GY_IIC_WaitAck()) //如果从机未应答则数据发送失败
	{
		GY_IIC_Stop();
		return 1;
	}
	GY_IIC_SendByte(reg); //发送寄存器地址
	GY_IIC_WaitAck();	  
	
	GY_IIC_Start();
	GY_IIC_SendByte(I2C_Addr+1); //进入接收模式			   
	GY_IIC_WaitAck();
	*buf=GY_IIC_ReadByte(0);	   
	GY_IIC_Stop(); //产生一个停止条件
	return 0;
}

/*************************************************************************************
* 函  数：uint8_t GY_IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr，uint8_t buf))
* 功　能：写入指定设备 指定寄存器的一个值
* 参  数：I2C_Addr  目标设备地址
		  reg	    寄存器地址
          buf       要写入的数据
* 返回值：1 失败 0成功
* 备  注：无
**************************************************************************************/ 
uint8_t GY_IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	GY_IIC_Start();
	GY_IIC_SendByte(I2C_Addr); //发送从机地址
	if(GY_IIC_WaitAck())
	{
		GY_IIC_Stop();
		return 1; //从机地址写入失败
	}
	GY_IIC_SendByte(reg); //发送寄存器地址
	GY_IIC_WaitAck();	  
	GY_IIC_SendByte(data); 
	if(GY_IIC_WaitAck())
	{
		GY_IIC_Stop(); 
		return 1; //数据写入失败
	}
	GY_IIC_Stop(); //产生一个停止条件
	return 0;
}

/***************************************************************************************
* 函  数：uint8_t GY_IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
* 功　能：读取指定设备 指定寄存器的 length个值
* 参  数：dev     目标设备地址
		  reg	  寄存器地址
          length  要读的字节数
		  *data   读出的数据将要存放的指针
* 返回值：1成功 0失败
* 备  注：无
***************************************************************************************/ 
uint8_t GY_IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;
	uint8_t temp;
	GY_IIC_Start();
	GY_IIC_SendByte(dev); //发送从机地址
	if(GY_IIC_WaitAck())
	{
		GY_IIC_Stop(); 
		return 1; //从机地址写入失败
	}
	GY_IIC_SendByte(reg); //发送寄存器地址
	GY_IIC_WaitAck();	  
	GY_IIC_Start();
	GY_IIC_SendByte(dev+1); //进入接收模式	
	GY_IIC_WaitAck();
	for(count=0;count<length;count++)
	{
		if(count!=(length-1))
			temp = GY_IIC_ReadByte(1); //带ACK的读数据
		else  
			temp = GY_IIC_ReadByte(0); //最后一个字节NACK

		data[count] = temp;
	}
  GY_IIC_Stop(); //产生一个停止条件
	return 0;
}

/****************************************************************************************
* 函  数：uint8_t GY_IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
* 功　能：将多个字节写入指定设备 指定寄存器
* 参  数：dev     目标设备地址
		  reg	  寄存器地址
		  length  要写的字节数
		  *data   要写入的数据将要存放的指针
* 返回值：1成功 0失败
* 备  注：无
****************************************************************************************/ 
uint8_t GY_IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
 	uint8_t count = 0;
	GY_IIC_Start();
	GY_IIC_SendByte(dev); //发送从机地址
	if(GY_IIC_WaitAck())
	{
		GY_IIC_Stop();
		return 1; //从机地址写入失败
	}
	GY_IIC_SendByte(reg); //发送寄存器地址
	GY_IIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		GY_IIC_SendByte(data[count]); 
		if(GY_IIC_WaitAck()) //每一个字节都要等从机应答
		{
			GY_IIC_Stop();
			return 1; //数据写入失败
		}
	}
	GY_IIC_Stop(); //产生一个停止条件
	return 0;
}





