/*******************************************************************************************
* �жϷ�������д
* ����汾��V1.0
* �������ڣ�2020-8-18
* �������ߣ����
*******************************************************************************************/

#include "main.h"

/******************************************************************************/
/*                                ȫ�ֱ���                                     */
/******************************************************************************/
uint8_t DAT_Scan = 0;
uint8_t CTR_Scan = 0;
uint8_t IMU_Scan = 0;
uint8_t MPU_Scan = 0;
uint8_t IRQ_Scan = 0;


extern u8  USART2_RX_BUF[200];

/******************************************************************************/
/*                                �ڲ�����                                     */
/******************************************************************************/
void NMI_Handler(void)
{
}
	//���󱨾�
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1){}
}
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1){}
}
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
    while (1){}
}
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
    while (1){}
}
void SVC_Handler(void)
{
}
void DebugMon_Handler(void)
{
}
void PendSV_Handler(void)
{
}
void SysTick_Handler(void)
{
}
/******************************************************************************/
/*                                ���躯��                                     */
/******************************************************************************/

void TIM4_IRQHandler(void)   //TIM4�жϷ�����
{
	static uint16_t ms5 = 0,ms10 = 0,ms20 = 0,ms100 = 0,ms200 = 0,ms800=0; //��Ƶϵ��
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)	//�ж��Ƿ����TIM�����ж�
	{
		
		ms5++;
		ms20++;
		ms10++;		
		ms100++;
		ms200++;
		ms800++;
		if(ms5 >= 5)//200Hz
		{
			ms5 = 0;
			MPU_Scan = 1;
		}
		if(ms10 >= 10)//100Hz
		{
			ms10 = 0;
			IMU_Scan = 1;
		}
		if(ms20 >= 20)//50Hz
		{
			ms20 = 0;
			DAT_Scan = 1;
		}
		if(ms800 >= 100)//
		{
			ms800 = 0;
			CTR_Scan = 1;
		}
		if(ms200 >= 200)//5Hz
		{
			ms200 = 0;
			IRQ_Scan = 1;
		}
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	//���TIM4�����ж�
}

void USART2_IRQHandler(void)                	//����2�жϷ������
{
	u8 Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART2);	//��ȡ���յ�������
		
		if((USART_RX_STA[1]&0x8000)==0)//����δ���
		{
			if(USART_RX_STA[1]&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA[1]=0;//���մ���,���¿�ʼ
				else USART_RX_STA[1]|=0x8000;	//��������� 
			}
			else //��û�յ�0X0d
			{	
				if(Res==0x0d)USART_RX_STA[1]|=0x4000;
				else
				{
					USART2_RX_BUF[USART_RX_STA[1]&0X3FFF]=Res ;
					USART_RX_STA[1]++;
					if(USART_RX_STA[1]>(200-1))USART_RX_STA[1]=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		} 
   } 			
} 

