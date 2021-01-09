#include "myiic.h"
#include "delay.h"
/////////////////////////////////////////////////////////////////////////////////////////		 
//B30 深度传感器测试例程
//开发板：BlueTest STM32
//交流论坛: www.Bluerobots.cn （BlueRobots 水下机器人社区）
//修改日期: 2019/4/30
//例程版本：V1.2
//联系邮箱：info@bluerobots.cn
//特别声明：本程序来源于网络，经BlueRobots 社区网友修改后用于交流分享，使用者自行承担一切后果！
/////////////////////////////////////////////////////////////////////////////////////////	

//初始化IIC
void GY_IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
		
	//关闭JTAG复用
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GY_IIC_SCL=1;
	GY_IIC_SDA=1;
}
//产生IIC起始信号
void GY_IIC_Start(void)
{
	GY_SDA_OUT();     //sda线输出
	GY_IIC_SDA=1;	
	GY_IIC_SCL=1;
	delay_us(5);
 	GY_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	GY_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void GY_IIC_Stop(void)
{
	GY_SDA_OUT();//sda线输出
	GY_IIC_SCL=0;
	GY_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(5);
	GY_IIC_SCL=1; 
	GY_IIC_SDA=1;//发送I2C总线结束信号
	delay_us(5);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 GY_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	GY_SDA_IN();      //SDA设置为输入  
	GY_IIC_SDA=1;delay_us(5);	   
	GY_IIC_SCL=1;delay_us(5);	 
	while(GY_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			GY_IIC_Stop();
			return 1;
		}
	}
	GY_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void GY_IIC_Ack(void)
{
	GY_IIC_SCL=0;
	GY_SDA_OUT();
	GY_IIC_SDA=0;
	delay_us(5);
	GY_IIC_SCL=1;
	delay_us(5);
	GY_IIC_SCL=0;
}
//不产生ACK应答		    
void GY_IIC_NAck(void)
{
	GY_IIC_SCL=0;
	GY_SDA_OUT();
	GY_IIC_SDA=1;
	delay_us(5);
	GY_IIC_SCL=1;
	delay_us(5);
	GY_IIC_SCL=0;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void GY_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  GY_SDA_OUT(); 	    
    GY_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        GY_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(5);   
		GY_IIC_SCL=1;
		delay_us(5); 
		GY_IIC_SCL=0;	
		delay_us(5);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 GY_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	GY_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        GY_IIC_SCL=0; 
        delay_us(5);
		GY_IIC_SCL=1;
        receive<<=1;
        if(GY_READ_SDA)receive++;   
		delay_us(5); 
    }					 
    if (!ack)
        GY_IIC_NAck();//发送nACK
    else
        GY_IIC_Ack(); //发送ACK   
    return receive;
}


// BlueRobots Lab
























