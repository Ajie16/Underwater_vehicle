#include "main.h"



void GY_Init(void)
{
	
}

void MS583703BA_RESET(void)
{
	GY_IIC_Start();
	GY_IIC_Send_Byte(MS583703BA_SlaveAddress);//CSB接地，主机地址：0XEE，否则 0X77
  GY_IIC_Wait_Ack();
  GY_IIC_Send_Byte(MS583703BA_RST);//发送复位命令
	GY_IIC_Wait_Ack();
  GY_IIC_Stop();
	
}

/**
  * @brief  MS5837_CRC4校验(4bit校验)【官方数据手册提供】
  * @param  MS5837 PROM标定参数数组
  * @retval 返回CRC校验码
  */
unsigned char MS5837_CRC4(unsigned int n_prom[]) // n_prom defined as 8x unsigned int (n_prom[8])
{
		int cnt; // simple counter
		unsigned int n_rem=0; // crc remainder
		unsigned char n_bit;
		n_prom[0]=((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
		n_prom[7]=0; // Subsidiary value, set to 0
		for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
		{ // choose LSB or MSB
				if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
				else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
				for (n_bit = 8; n_bit > 0; n_bit--)
				{
						if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
						else n_rem = (n_rem << 1);
				}
		}
		n_rem= ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
		return (n_rem ^ 0x00);
}


/**
  * @brief  MS5837_Init
  * @param  None
  * @retval 返回MS5837初始化是否成功标志：1成功，0失败
  */

u8 MS5837_Init(void)
{	 
		u8  inth,intl,i;
		u8 CRC_Check = 0;
		for (i=0;i<=6;i++) 
		{
				GY_IIC_Start();
				GY_IIC_Send_Byte(MS583703BA_SlaveAddress);
				GY_IIC_Wait_Ack();
				GY_IIC_Send_Byte(0xA0 + (i*2));
				GY_IIC_Wait_Ack();
				GY_IIC_Stop();

				rt_hw_us_delay(5);
				GY_IIC_Start();
				GY_IIC_Send_Byte(MS583703BA_SlaveAddress+0x01);  //进入接收模式

				rt_hw_us_delay(1);
				GY_IIC_Wait_Ack();
				inth = GY_IIC_Read_Byte(1);  		//带ACK的读数据
				
				rt_hw_us_delay(1);
				intl = IIC_Read_Byte(0); 			//最后一个字节NACK
				IIC_Stop();
				Cal_C[i] = (((uint16_t)inth << 8) | intl);
				rt_kprintf("Cal_C[%d]:%d\r\n",i,Cal_C[i]);
		}
		CRC_Check = (u8)((Cal_C[0]&0xF000)>>12);
		rt_kprintf("CRC:%d   CRC_Check:%d\r\n",MS5837_CRC4(Cal_C),CRC_Check);

		if(CRC_Check == MS5837_CRC4(Cal_C)){

				return true;
		}
		else 
		{return false;}
}

/**
  * @brief  MS583703BA转换温度结果
  * @param  None
  * @retval None
  */
void MS583703BA_getTemperature(void)
{
	D2_Temp = MS583703BA_getConversion(MS583703BA_D2_OSR_2048);//4096  出现周期性尖峰(300+)     
	
	dT=D2_Temp - (((uint32_t)Cal_C[5])*256);
	MS_TEMP=2000+dT*((uint32_t)Cal_C[6])/8388608;  //问题在于此处没有出现负号
}

/**
  * @brief  MS583703BA转换气压结果
  * @param  None
  * @retval None
  */
void MS583703BA_getPressure(void)
{
		D1_Pres= MS583703BA_getConversion(MS583703BA_D1_OSR_8192);//2048
		
		OFF_=(uint32_t)Cal_C[2]*65536+((uint32_t)Cal_C[4]*dT)/128;
		SENS=(uint32_t)Cal_C[1]*32768+((uint32_t)Cal_C[3]*dT)/256;
		if(MS_TEMP<2000)  // LOW Temperature
		{
				Aux = (2000-MS_TEMP)*(2000-MS_TEMP);
				T2 = 3*(dT*dT) /0x80000000; 
				OFF2 = (uint32_t)1.5*Aux;
				SENS2 = 5*Aux/8;
				
				OFF_ = OFF_ - OFF2;
				SENS = SENS - SENS2;	
		}
		else{
				Aux = (2000-MS_TEMP)*(2000-MS_TEMP);
				T2=2*(dT*dT)/137438953472;
				OFF2 = 1*Aux/16;
				SENS2 = 0;
				OFF_ = OFF_ - OFF2;
				SENS = SENS - SENS2;	
			 
		}
		MS5837_Pressure = ((D1_Pres*SENS/2097152-OFF_)/8192)/10;
		MS_TEMP = (MS_TEMP-T2)/100;

}




