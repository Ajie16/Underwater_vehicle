#ifndef   _LED_H
#define   _LED_H


/***************************************************************************
* �궨���� PB0-�� PB1-��
***************************************************************************/
//#define LED_GREEN_ON() GPIO_SetBits(GPIOA,GPIO_Pin_11)
//#define LED_GREEN_OFF() GPIO_ResetBits(GPIOA,GPIO_Pin_11)

#define LED_RED_ON() GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define LED_RED_OFF() GPIO_ResetBits(GPIOB,GPIO_Pin_7)

void LED_Init(void);

#endif

