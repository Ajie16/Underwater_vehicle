#include "myiic.h"
#include "delay.h"
/////////////////////////////////////////////////////////////////////////////////////////		 
//B30 ��ȴ�������������
//�����壺BlueTest STM32
//������̳: www.Bluerobots.cn ��BlueRobots ˮ�»�����������
//�޸�����: 2019/4/30
//���̰汾��V1.2
//��ϵ���䣺info@bluerobots.cn
//�ر���������������Դ�����磬��BlueRobots ���������޸ĺ����ڽ�������ʹ�������ге�һ�к����
/////////////////////////////////////////////////////////////////////////////////////////	

//��ʼ��IIC
void GY_IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
		
	//�ر�JTAG����
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GY_IIC_SCL=1;
	GY_IIC_SDA=1;
}
//����IIC��ʼ�ź�
void GY_IIC_Start(void)
{
	GY_SDA_OUT();     //sda�����
	GY_IIC_SDA=1;	
	GY_IIC_SCL=1;
	delay_us(5);
 	GY_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	GY_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void GY_IIC_Stop(void)
{
	GY_SDA_OUT();//sda�����
	GY_IIC_SCL=0;
	GY_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(5);
	GY_IIC_SCL=1; 
	GY_IIC_SDA=1;//����I2C���߽����ź�
	delay_us(5);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 GY_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	GY_SDA_IN();      //SDA����Ϊ����  
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
	GY_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void GY_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	  GY_SDA_OUT(); 	    
    GY_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
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
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 GY_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	GY_SDA_IN();//SDA����Ϊ����
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
        GY_IIC_NAck();//����nACK
    else
        GY_IIC_Ack(); //����ACK   
    return receive;
}


// BlueRobots Lab
























