/////////////////////////////////////////////////////////////////////////////////////////		 
//B30 ��ȴ�������������
//�����壺BlueTest STM32
//������̳: www.Bluerobots.cn ��BlueRobots ˮ�»�����������
//�޸�����: 2019/4/30
//���̰汾��V1.2
//��ϵ���䣺info@bluerobots.cn
//�ر���������������Դ�����磬��BlueRobots ���������޸ĺ����ڽ���������ʹ�������ге�һ�к����
/////////////////////////////////////////////////////////////////////////////////////////	
#include "MS5837.h"
#include "math.h"
#include "myiic.h"



 
/*
C1  ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3	�¶�ѹ��������ϵ�� TCS
C4	�¶�ϵ����ѹ������ TCO
C5	�ο��¶� T|REF
C6 	�¶�ϵ�����¶� TEMPSENS
*/
uint32_t Cal_C[7];        //���ڴ��PROM�е�6������1-6


float Aux;
/*
dT ʵ�ʺͲο��¶�֮��Ĳ���
TEMP ʵ���¶�	
*/
signed int dT,TEMP;
/*
OFF ʵ���¶Ȳ���
SENS ʵ���¶�������
*/
int64_t OFF_,SENS;
uint32_t D1_Pres,D2_Temp;	// ����ѹ��ֵ,�����¶�ֵ
double Temperature;
int32_t Pressure,Pressure_old;				//����ѹ
int32_t OFFi=0,SENSi=0,Ti=0;
int64_t OFF2 = 0;
int64_t SENS2 = 0;
uint32_t TEMP2;	//�¶�У��ֵ
double T2;





/*******************************************************************************
  * @��������	  MS5837BA_RESET
  * @����˵��   ��λMS5837
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
*******************************************************************************/
void MS5837_30BA_ReSet(void)
{
		GY_IIC_Start();
		GY_IIC_Send_Byte(MS5837_30BA_WriteCommand);//����д����
	  GY_IIC_Wait_Ack();
    GY_IIC_Send_Byte(MS5837_30BA_ResetCommand);//���͸�λ����
	  GY_IIC_Wait_Ack();
    GY_IIC_Stop();
	
}
/*******************************************************************************
  * @��������	  MS5837_PROM
  * @����˵��   ��ʼ��5837_30BA
  * @�������  	��
  * @�������   ��
  * @���ز���   ��
  * @ȫ�ֱ���   Cal_C����,����CRCУ���Լ�����ѹ��ֵ���¶�ֵ
*******************************************************************************/
u8 MS5837_30BA_PROM(void)
 {	 
  u8  inth,intl;
  int i;

	MS5837_30BA_ReSet();	                                             //��λMS5837
  delay_ms(20); 
	for (i=0;i<7;i++) 
	{
		GY_IIC_Start();
    GY_IIC_Send_Byte(MS5837_30BA_WriteCommand);      //����д����
		GY_IIC_Wait_Ack();
		GY_IIC_Send_Byte(MS5837_30BA_PROM_RD + (i*2));   //����PROM��ȡ����
		GY_IIC_Wait_Ack();
    GY_IIC_Stop();
		delay_us(5);
		GY_IIC_Start();
		GY_IIC_Send_Byte(MS5837_30BA_ReadCommand);       //�������ģʽ
		delay_us(5);
		GY_IIC_Wait_Ack();
		inth = GY_IIC_Read_Byte(1);  		                //��ACK�Ķ�����
		delay_us(5);
		intl = GY_IIC_Read_Byte(0); 			                //���һ���ֽ�NACK
		GY_IIC_Stop();
    Cal_C[i] = (((uint16_t)inth << 8) | intl);
	}
	 return !Cal_C[0];
 }


/**************************ʵ�ֺ���********************************************
*����ԭ��:    unsigned long MS5837BA_getConversion(void)
*��������:    ��ȡ MS5837 ��ת����� 
*******************************************************************************/
unsigned long MS5837_30BA_GetConversion(u8 command)
{
      
			unsigned long conversion = 0;
			u8 temp[3];
	    GY_IIC_Start();
			GY_IIC_Send_Byte(MS5837_30BA_WriteCommand); 		//д��ַ
			GY_IIC_Wait_Ack();
			GY_IIC_Send_Byte(command); //дת������
			GY_IIC_Wait_Ack();
			GY_IIC_Stop();

			delay_ms(10);
			GY_IIC_Start();
			GY_IIC_Send_Byte(MS5837_30BA_WriteCommand); 		//д��ַ
			GY_IIC_Wait_Ack();
			GY_IIC_Send_Byte(0);				// start read sequence
			GY_IIC_Wait_Ack();
			GY_IIC_Stop();
		 
			GY_IIC_Start();
			GY_IIC_Send_Byte(MS5837_30BA_ReadCommand);  //�������ģʽ
			GY_IIC_Wait_Ack();
			temp[0] = GY_IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 23-16
			temp[1] = GY_IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 8-15
			temp[2] = GY_IIC_Read_Byte(0);  //��NACK�Ķ����� bit 0-7
			GY_IIC_Stop();
			
			conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];
			return conversion;
 
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:    void MS5837_30BA_GetData(void)
*��������:    ��ȡ�¶�ѹ�����ݲ����ж����¶Ȳ���
*******************************************************************************/

void MS5837_30BA_GetData(void)
{
	
	D2_Temp = MS5837_30BA_GetConversion(MS5837_30BA_D2_OSR_8192);
	delay_ms(20);
	D1_Pres= MS5837_30BA_GetConversion(MS5837_30BA_D1_OSR_8192);
	delay_ms(20);
	dT=D2_Temp - (((uint32_t)Cal_C[5])*256l);
	SENS=(int64_t)Cal_C[1]*32768l+((int64_t)Cal_C[3]*dT)/256l;
	OFF_=(int64_t)Cal_C[2]*65536l+((int64_t)Cal_C[4]*dT)/128l;
	TEMP = 2000l+(int64_t)(dT)*Cal_C[6]/8388608LL;
	
//�����¶Ȳ���
	if(TEMP<2000)  // low temp
	{
		Ti = (3*(int64_t)(dT)*(int64_t)(dT)/(8589934592LL));
		OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
		SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
	}
	 else{         // high temp
	  Ti = 2*(dT*dT)/(137438953472LL);
		OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
		SENSi = 0;
	 }
  OFF2 = OFF_-OFFi;           
	SENS2 = SENS-SENSi;
  Pressure=((D1_Pres*SENS2)/2097152l-OFF2)/8192l/10;          //У׼��ѹ������
  Temperature=(TEMP-Ti)/100.0;                                //У׼���¶�����  
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:    unsigned char MS5837_30BA_Crc4()
*��������:    �Ի�ȡ��Cal_C�������CRCУ��
*******************************************************************************/

unsigned char MS5837_30BA_Crc4()
{
	int cnt;
	int t;
	unsigned int n_rem=0;
	unsigned char n_bit;	
	unsigned char  a=0;
	unsigned char  b=0;
	unsigned short  int n_prom[8];
	
	for( t=0;t<7;t++)
{
   n_prom[t]=Cal_C[t];
}	
	n_prom[0]=((n_prom[0]) & 0x0FFF);
	n_prom[7]=0;
	for (cnt = 0; cnt < 16; cnt++)
		{
	if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);	
	else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);		
	for (n_bit = 8; n_bit > 0; n_bit--)
	{
	if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
	else n_rem = (n_rem << 1);
	}

		}
	n_rem= ((n_rem >> 12) & 0x000F);	
    a=(n_rem ^ 0x00);
	   b=Cal_C[0]>>12;
		if (a==b)
		{
		   return 1;
		}
		else return 0;
	
}