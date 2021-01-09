/*******************************************************************************************
* BSP初始化以及任务调度
* 程序版本：V1.0
* 程序日期：2020-8-18
* 程序作者：徐杰
*******************************************************************************************/
#include "main.h"

//外部变量
extern u8  USART2_RX_BUF[USART_REC_LEN];
extern int32_t Pressure;
extern uint32_t D1_Pres,D2_Temp;	// 数字压力值,数字温度值
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

void System_Init(void)	//初始化
{
	//中断管理
	NvicConfig();
	//延时函数初始化
	delay_init();
	//时基初始化
	TIM_Init();
	//LED初始化
	LED_Init();
	//串口初始化
	uart_init(9600);
	usart2_init(115200);
	//OLED初始化
	OLED_Init();
	//MPU6050初始化
	MPU6050_Init();
	//电机驱动初始化 
	MOTOR_Init();
	//msiic初始化
	GY_IIC_Init();
	MS5837_30BA_PROM();
	//pid初始化
	DefaultParams();
	TableToParams();
	PidParameter_init();
	//电调初始化
	Moto_Pwm(190,190,190,190);
	TIM1->CCR1 = 100;
	TIM1->CCR4 = 100;
	delay_ms(1000); 
	Moto_Pwm(100,100,100,100);
	TIM1->CCR1 = 190;
	TIM1->CCR4 = 190;
	//测试水深目标
	alt_target=1010;
}

void Task_Schedule(void)
{
  	
		if(IMU_Scan) //100Hz
		{
			//姿态读取与解算
			IMU_Scan  = 0;
			OLED_Refresh_Gram();
//			Prepare_Data(); //获取姿态解算所需数据
//			IMUupdate(&Gyr_rad,&Acc_filt,&Att_Angle); //四元数姿态解算
//			if(STATE_CTR==1)
//			  Control(&Att_Angle,&Gyr_rad);           //姿态控制
		}
		
		if(DAT_Scan) //50Hz
		{
			DAT_Scan = 0;
			//数据打印
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
			//预扩展中断
			MS5837_30BA_GetData();
			OLED_ShowNum(48,48,Pressure,5,16);
			OLED_ShowNum(50,0,alt_target,5,16);
		
		}
		//接收到完整数据的处理函数（串口1）
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
				//重新设立目标
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
				//死程序
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
		//串口2
			if(USART_RX_STA[1]&0x8000)
		{
			
      X=(USART2_RX_BUF[1]-'0')*100+(USART2_RX_BUF[2]-'0')*10+(USART2_RX_BUF[3]-'0');//位移差
			J=(USART2_RX_BUF[5]-'0')*100+(USART2_RX_BUF[6]-'0')*10+(USART2_RX_BUF[7]-'0');//角度差
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



