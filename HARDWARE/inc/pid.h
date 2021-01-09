#ifndef __PID_H
#define __PID_H

#include "imu.h"


typedef struct  
{ 
	float kp,ki,kd,
	outP,outI,outD,
	error,prevError,
	integ,deriv,dt,
	iLimit,iLimitLow;
}PidObject;

void PidParameter_init(void);
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure);
void DefaultParams(void);
void TableToParams(void);
float pidUpdate(PidObject* pid,float error);

#endif
