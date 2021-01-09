#ifndef   _CONTROL_H
#define   _CONTROL_H

#include "stm32f10x.h"
#include "imu.h"

void Control(FLOAT_ANGLE *att_in,FLOAT_XYZ *gyr_in);
void right(void);
void left(void);
void qianjin(void);

#endif

