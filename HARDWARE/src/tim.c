/*******************************************************************************************
* ʱ��ʱ��TIM4
* ����汾��V1.0
* �������ڣ�2020-8-18
* �������ߣ����
*******************************************************************************************/
#include "stm32f10x.h"


/*****************************************************************************
* ��  ����void TIM_Init(void)
* ��  �ܣ�TIM4��ʼ��Ϊ1ms����һ��
* ��  ������
* ����ֵ����
* ��  ע�������ж�ʱ�� Tout = (ARR-1)*(PSC-1)/CK_INT
*****************************************************************************/
void TIM_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;   //���嶨ʱ���ṹ�����
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);   //ʹ��TIM4��ʱ��
	
	TIM_TimeBaseInitStruct.TIM_Period=720-1;   //�����Զ���װ�ص�����ֵ
	TIM_TimeBaseInitStruct.TIM_Prescaler=100-1;   //����Ԥ��Ƶֵ
	TIM_TimeBaseInitStruct.TIM_ClockDivision=0;   //����ʱ�ӷָ�
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;   //���ϼ���ģʽ
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);   //��ʱ����ʼ������
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);   //TIM4�ж�ʹ��
	
	TIM_Cmd(TIM4,ENABLE);   //TIM4ʹ��
}