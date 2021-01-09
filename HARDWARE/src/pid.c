/*******************************************************************************************
* PID
* 程序版本：V1.0
* 程序日期：2020-9-7
* 程序作者：徐杰
*******************************************************************************************/
#include "main.h"

PID_SAVE PIDflash;
extern s16 X,J,duoxiao,out_w,out_n;

PidObject pid_n,pid_w;

void DefaultParams(void)
{ 	
	//零偏数据
	PIDflash.ACC_OFFSET_X  = 0;
	PIDflash.ACC_OFFSET_Y  = 0;
	PIDflash.ACC_OFFSET_Z  = 0;
	PIDflash.GYRO_OFFSET_X = 0;
	PIDflash.GYRO_OFFSET_Y = 0;
	PIDflash.GYRO_OFFSET_Z = 0;
	
	//角度环数据PID参数
	PIDflash.ROL_Angle_P = 1200;
	PIDflash.ROL_Angle_I = 8;
	PIDflash.ROL_Angle_D = 50;
	PIDflash.PIT_Angle_P = 1200;
	PIDflash.PIT_Angle_I = 8;
	PIDflash.PIT_Angle_D = 50;
	
	//角速度环PID参数
	PIDflash.ROL_Rate_P  = 630;
	PIDflash.ROL_Rate_I  = 5;
	PIDflash.ROL_Rate_D  = 300;
	
	PIDflash.PIT_Rate_P  = 630;
	PIDflash.PIT_Rate_I  = 5;
	PIDflash.PIT_Rate_D  = 300;
	
	//高度环PID
	PIDflash.ALT_Rate_P = 1200;
	PIDflash.ALT_Rate_I = 5;
	PIDflash.ALT_Rate_D = 50;           	
}


void TableToParams(void)
{
	//零偏数据
	ACC_OFFSET_RAW.X = PIDflash.ACC_OFFSET_X;
	ACC_OFFSET_RAW.Y = PIDflash.ACC_OFFSET_Y;
	ACC_OFFSET_RAW.Z = PIDflash.ACC_OFFSET_Z;
	GYRO_OFFSET_RAW.X = PIDflash.GYRO_OFFSET_X;
	GYRO_OFFSET_RAW.Y = PIDflash.GYRO_OFFSET_Y;
	GYRO_OFFSET_RAW.Z = PIDflash.GYRO_OFFSET_Z;
	
	//角度环数据PID参数
	PID_ROL_Angle.P = PIDflash.ROL_Angle_P/1000.0f;
	PID_ROL_Angle.I = PIDflash.ROL_Angle_I/1000.0f;
	PID_ROL_Angle.D = PIDflash.ROL_Angle_D/1000.0f;
	PID_PIT_Angle.P = PIDflash.PIT_Angle_P/1000.0f;
	PID_PIT_Angle.I = PIDflash.PIT_Angle_I/1000.0f;
	PID_PIT_Angle.D = PIDflash.PIT_Angle_D/1000.0f;
		
	//角速度环PID参数	
	PID_ROL_Rate.P = PIDflash.ROL_Rate_P/1000.0f;
	PID_ROL_Rate.I = PIDflash.ROL_Rate_I/1000.0f;
	PID_ROL_Rate.D = PIDflash.ROL_Rate_D/1000.0f;
	PID_PIT_Rate.P = PIDflash.PIT_Rate_P/1000.0f;
	PID_PIT_Rate.I = PIDflash.PIT_Rate_I/1000.0f;
	PID_PIT_Rate.D = PIDflash.PIT_Rate_D/1000.0f;

	//高度环PID
	PID_ALT_Rate.P = PIDflash.ALT_Rate_P/1000.0f;
	PID_ALT_Rate.I = PIDflash.ALT_Rate_I/1000.0f;
	PID_ALT_Rate.D = PIDflash.ALT_Rate_D/1000.0f;

}

/*****************************************************************************
* 函  数：void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
* 功  能：位置式PID算法
* 参  数：PID: 算法P I D参数的结构体
*         target: 目标值
*         measure: 测量值
* 返回值：无
* 备  注: 角度环和角速度环共用此函数
*****************************************************************************/
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{
	PID->Error  = target - measure;              //误差
	PID->Differ = PID->Error - PID->PreError;    //微分量
	PID->Pout = PID->P * PID->Error;                        //比例控制
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //积分控制
	PID->Dout = PID->D * PID->Differ;                       //微分控制
	PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //比例 + 积分 + 微分总控制
	PID->Integral += PID->Error;                        //对误差进行积分
	if(PID->Integral > PID->Irang)                      //积分限幅
		PID->Integral = PID->Irang;
	if(PID->Integral < -PID->Irang)                     //积分限幅
		PID->Integral = -PID->Irang;                    
	PID->PreError = PID->Error ;                            //前一个误差值
}

/*****************************************************************************
* 函  数：void PidParameter_init(void)
* 功  能：初始化PID结构体里的一些成员值
* 参  数：无
* 返回值：无 
* 备  注: 由于PID参数我们都保存在FLASH中，所以此函数初始化时不用初始化这些参数，
*         但是Flash中的参数有可能因为误操作被擦除，如果Flash读取参数失败，则就
*         初始化为默认参数。
*****************************************************************************/
void PidParameter_init(void)
{
	//ROLL轴
	PID_ROL_Rate.Ilimit_flag = 1; //Roll轴角速度积分的分离标志
	PID_ROL_Rate.Ilimit = 150;    //Roll轴角速度积分范围
	PID_ROL_Rate.Irang = 1200;    //Roll轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_ROL_Angle.Ilimit_flag= 1; //Roll轴角度积分的分离标志
	PID_ROL_Angle.Ilimit = 35;    //Roll轴角度积分范围
	PID_ROL_Angle.Irang = 200;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）

	//PITCH轴
	PID_PIT_Rate.Ilimit_flag = 1; //Pitch轴角速度积分的分离标志
	PID_PIT_Rate.Ilimit = 150;    //Pitch轴角速度积分范围
	PID_PIT_Rate.Irang = 1200;    //Pitch轴角速度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	PID_PIT_Angle.Ilimit_flag = 1;//Roll轴角度积分的分离标志
	PID_PIT_Angle.Ilimit = 35;    //Roll轴角度积分范围
	PID_PIT_Angle.Irang = 200;    //Roll轴角度积分限幅度（由于电机输出有限，所以积分输出也是有限的）
	//高度环
	PID_ALT_Rate.Ilimit_flag = 1;
	PID_ALT_Rate.Ilimit = 5;
	PID_ALT_Rate.Irang = 20;
	//坐标环
	pid_n.dt=1;
	pid_n.error=0;
	pid_n.iLimit=0;
	pid_n.iLimitLow=0;
	pid_n.integ=0;
	pid_n.kd=0;//
	pid_n.ki=0;
	pid_n.kp=2.5;//
	pid_n.prevError=0;
	//前进角度环
	pid_w.dt=1;
	pid_w.error=0;
	pid_w.iLimit=0;
	pid_w.iLimitLow=0;
	pid_w.integ=0;
	pid_w.kd=1;//
	pid_w.ki=0;
	pid_w.kp=3;//
	pid_w.prevError=0;	
	
}
//--------前进角度PID结构--------------//
float pidUpdate(PidObject* pid,float error)
{
	float output;
	
	pid->error = error;
	pid->integ += pid->error * pid->dt;
	
	if (pid->integ > pid->iLimit)
	{
		pid->integ = pid->iLimit;
	}
	else if (pid->integ < pid->iLimitLow)
	{
		pid->integ = pid->iLimitLow;
	}
	pid->deriv = (pid->error - pid->prevError) / pid->dt;
	
	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;
	
	output = pid->outP + pid->outI + pid->outD; 
	pid->prevError = pid->error; 
	
	return output;
}



