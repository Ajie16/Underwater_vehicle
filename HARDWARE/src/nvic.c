/*******************************************************************************************
* NVIC-ALL�жϹ������
* ����汾��V1.0
* �������ڣ�2020-8-18
* �������ߣ����
*******************************************************************************************/
#include "stm32f10x.h"

/*****************************************************************************
* ��  ����void NvicConfig(void)
* ��  �ܣ����ù����������жϵ����ȼ�
* ��  ������
* ����ֵ����
* ��  ע�������ȼ��жϲ�Ҫ������Ŷ
*****************************************************************************/
void NvicConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM4_IRQn;   //TIM4�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;   //��ռ���ȼ�0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;   //�����ȼ�1
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;   //ʹ��TIM4�ж�ͨ��
	NVIC_Init(&NVIC_InitStruct);   //�ж����ȼ���ʼ������
	
	  //Usart1 NVIC ����
  NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�3
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStruct);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI15_10_IRQn;   //�����ⲿ�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;   //������ռ���ȼ�Ϊ0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;   //���������ȼ�Ϊ1
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;   //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStruct);   //�ж����ȼ���ʼ������
}

