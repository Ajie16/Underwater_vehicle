/*******************************************************************************************
* BSP��ʼ���Լ��������
* ����汾��V1.0
* �������ڣ�2020-8-18
* �������ߣ����
*******************************************************************************************/
#include "main.h"

//�ⲿ����
extern u8  USART2_RX_BUF[USART_REC_LEN];
extern int32_t Pressure;
extern uint32_t D1_Pres,D2_Temp;	// ����ѹ��ֵ,�����¶�ֵ
extern uint32_t alt_target;
extern u8 CTR_Scan;
extern float angle_rol;
extern float angle_pit;
u8 set_zhi=170;

u8 STATE_CTR,STATE_RUN,STATE_R,i,j;
u16 receive_dat[3];
s16 X,J,correct,out_w,out_n;
extern PidObject pid_n,pid_w;

u16 x=2000,y=1000,z=1000;

void System_Init(void)	//��ʼ��
{
	//�жϹ���
	NvicConfig();
	//��ʱ������ʼ��
	delay_init();
	//ʱ����ʼ��
	TIM_Init();
	//LED��ʼ��
	LED_Init();
	//���ڳ�ʼ��
	uart_init(9600);
	usart2_init(115200);
	//OLED��ʼ��
	OLED_Init();
	//MPU6050��ʼ��
	MPU6050_Init();
	//���������ʼ�� 
	MOTOR_Init();
	//msiic��ʼ��
	GY_IIC_Init();
	MS5837_30BA_PROM();
	//pid��ʼ��
	DefaultParams();
	TableToParams();
	PidParameter_init();
	//�����ʼ��
	Moto_Pwm(190,190,190,190);
	TIM1->CCR1 = 100;
	TIM1->CCR4 = 100;
	delay_ms(1000); 
	Moto_Pwm(100,100,100,100);
	TIM1->CCR1 = 190;
	TIM1->CCR4 = 190;
	//����ˮ��Ŀ��
	alt_target=1010;
}

void Task_Schedule(void)
{
  	
		if(IMU_Scan) //100Hz
		{
			//��̬��ȡ�����
			IMU_Scan  = 0;
			OLED_Refresh_Gram();
//			Prepare_Data(); //��ȡ��̬������������
//			IMUupdate(&Gyr_rad,&Acc_filt,&Att_Angle); //��Ԫ����̬����
//			if(STATE_CTR==1)
//			  Control(&Att_Angle,&Gyr_rad);           //��̬����
		}
		
		if(DAT_Scan) //50Hz
		{
			DAT_Scan = 0;
			//���ݴ�ӡ
			OLED_ShowNum(10,32,Att_Angle.pit,3,16);
			OLED_ShowNum(10,48,Att_Angle.rol,3,16);
			OLED_ShowNum(40,32,Att_Angle.yaw,3,16);
		}
		
		if(CTR_Scan) //800ms
		{
			CTR_Scan=0;  
			if(STATE_RUN==1)
			{
					TIM1->CCR1=set_zhi+correct;
					TIM1->CCR4=set_zhi-correct;
			}
			
		}
		if(IRQ_Scan) //5Hz
		{
			IRQ_Scan=0;
			//Ԥ��չ�ж�
			MS5837_30BA_GetData();
			OLED_ShowNum(48,48,Pressure,5,16);
			OLED_ShowNum(50,0,alt_target,5,16);
		
		}
		//���յ��������ݵĴ�����������1��
		if(USART_RX_STA[0]&0x8000)
		{
			OLED_ShowString(10,16,USART1_RX_BUF,16);
			if(USART1_RX_BUF[0]=='A')
				Moto_Pwm(190,190,190,190);
			else if(USART1_RX_BUF[0]=='B')
				Moto_Pwm(150,150,150,150);
			else if(USART1_RX_BUF[0]=='C')
			{
				Moto_Pwm(100,100,100,100);
				TIM1->CCR1 = 190;
				TIM1->CCR4 = 190;
				//��������Ŀ��
				angle_pit=Att_Angle.pit;
				angle_rol=Att_Angle.rol;
			}
			else if(USART1_RX_BUF[0]=='S')
			{
				STATE_CTR=0;
				STATE_RUN=0;
			}
			else if(USART1_RX_BUF[0]=='R')
				STATE_CTR=1;
			else if(USART1_RX_BUF[0]=='Q')
				qianjin();
			else if(USART1_RX_BUF[0]=='X')
				y+=100;
			else if(USART1_RX_BUF[0]=='Y')
				y-=100;
			else if(USART1_RX_BUF[0]=='M')
				z+=100;
			else if(USART1_RX_BUF[0]=='M')
				z-=100;
			else if(USART1_RX_BUF[0]=='N')
				x +=100;
			else if(USART1_RX_BUF[0]=='K')
				x -=100;
			else if(USART1_RX_BUF[0]=='L')
				STATE_RUN=1;
			else if(USART1_RX_BUF[0]=='E')
				set_zhi+=2;
			else if(USART1_RX_BUF[0]=='F')
				set_zhi-=2;
			else if(USART1_RX_BUF[0]=='O')
			{
					TIM1->CCR1=100;
					TIM1->CCR4=190;
			}
			else if(USART1_RX_BUF[0]=='G')
			{
				//������
				qianjin();
				delay_ms(x);
				left();
				delay_ms(y);
				qianjin();
				delay_ms(z);
				right();
				delay_ms(y);
				qianjin();
			}

			else
				Moto_Pwm(100,100,100,100);
			
			USART_RX_STA[0]=0;
		}
		//����2
			if(USART_RX_STA[1]&0x8000)
		{
			
      X=(USART2_RX_BUF[1]-'0')*100+(USART2_RX_BUF[2]-'0')*10+(USART2_RX_BUF[3]-'0');//λ�Ʋ�
			J=(USART2_RX_BUF[5]-'0')*100+(USART2_RX_BUF[6]-'0')*10+(USART2_RX_BUF[7]-'0');//�ǶȲ�
			J-=90;
			X-=90;		
			out_w=pidUpdate(&pid_w,X);
			out_n=pidUpdate(&pid_n,J);
			correct=out_w+out_n;
			if(correct>15)
			{
				correct=15;
			}
			if(correct<-15)
			{
				correct=-15;
			}
			USART_RX_STA[1]=0;
		}
}



