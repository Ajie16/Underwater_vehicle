/*******************************************************************************************
* PID
* ����汾��V1.0
* �������ڣ�2020-9-7
* �������ߣ����
*******************************************************************************************/
#include "main.h"

PID_SAVE PIDflash;
extern s16 X,J,duoxiao,out_w,out_n;

PidObject pid_n,pid_w;

void DefaultParams(void)
{ 	
	//��ƫ����
	PIDflash.ACC_OFFSET_X  = 0;
	PIDflash.ACC_OFFSET_Y  = 0;
	PIDflash.ACC_OFFSET_Z  = 0;
	PIDflash.GYRO_OFFSET_X = 0;
	PIDflash.GYRO_OFFSET_Y = 0;
	PIDflash.GYRO_OFFSET_Z = 0;
	
	//�ǶȻ�����PID����
	PIDflash.ROL_Angle_P = 1200;
	PIDflash.ROL_Angle_I = 8;
	PIDflash.ROL_Angle_D = 50;
	PIDflash.PIT_Angle_P = 1200;
	PIDflash.PIT_Angle_I = 8;
	PIDflash.PIT_Angle_D = 50;
	
	//���ٶȻ�PID����
	PIDflash.ROL_Rate_P  = 630;
	PIDflash.ROL_Rate_I  = 5;
	PIDflash.ROL_Rate_D  = 300;
	
	PIDflash.PIT_Rate_P  = 630;
	PIDflash.PIT_Rate_I  = 5;
	PIDflash.PIT_Rate_D  = 300;
	
	//�߶Ȼ�PID
	PIDflash.ALT_Rate_P = 1200;
	PIDflash.ALT_Rate_I = 5;
	PIDflash.ALT_Rate_D = 50;           	
}


void TableToParams(void)
{
	//��ƫ����
	ACC_OFFSET_RAW.X = PIDflash.ACC_OFFSET_X;
	ACC_OFFSET_RAW.Y = PIDflash.ACC_OFFSET_Y;
	ACC_OFFSET_RAW.Z = PIDflash.ACC_OFFSET_Z;
	GYRO_OFFSET_RAW.X = PIDflash.GYRO_OFFSET_X;
	GYRO_OFFSET_RAW.Y = PIDflash.GYRO_OFFSET_Y;
	GYRO_OFFSET_RAW.Z = PIDflash.GYRO_OFFSET_Z;
	
	//�ǶȻ�����PID����
	PID_ROL_Angle.P = PIDflash.ROL_Angle_P/1000.0f;
	PID_ROL_Angle.I = PIDflash.ROL_Angle_I/1000.0f;
	PID_ROL_Angle.D = PIDflash.ROL_Angle_D/1000.0f;
	PID_PIT_Angle.P = PIDflash.PIT_Angle_P/1000.0f;
	PID_PIT_Angle.I = PIDflash.PIT_Angle_I/1000.0f;
	PID_PIT_Angle.D = PIDflash.PIT_Angle_D/1000.0f;
		
	//���ٶȻ�PID����	
	PID_ROL_Rate.P = PIDflash.ROL_Rate_P/1000.0f;
	PID_ROL_Rate.I = PIDflash.ROL_Rate_I/1000.0f;
	PID_ROL_Rate.D = PIDflash.ROL_Rate_D/1000.0f;
	PID_PIT_Rate.P = PIDflash.PIT_Rate_P/1000.0f;
	PID_PIT_Rate.I = PIDflash.PIT_Rate_I/1000.0f;
	PID_PIT_Rate.D = PIDflash.PIT_Rate_D/1000.0f;

	//�߶Ȼ�PID
	PID_ALT_Rate.P = PIDflash.ALT_Rate_P/1000.0f;
	PID_ALT_Rate.I = PIDflash.ALT_Rate_I/1000.0f;
	PID_ALT_Rate.D = PIDflash.ALT_Rate_D/1000.0f;

}

/*****************************************************************************
* ��  ����void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
* ��  �ܣ�λ��ʽPID�㷨
* ��  ����PID: �㷨P I D�����Ľṹ��
*         target: Ŀ��ֵ
*         measure: ����ֵ
* ����ֵ����
* ��  ע: �ǶȻ��ͽ��ٶȻ����ô˺���
*****************************************************************************/
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure)
{
	PID->Error  = target - measure;              //���
	PID->Differ = PID->Error - PID->PreError;    //΢����
	PID->Pout = PID->P * PID->Error;                        //��������
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  //���ֿ���
	PID->Dout = PID->D * PID->Differ;                       //΢�ֿ���
	PID->OutPut =  PID->Pout + PID->Iout + PID->Dout;       //���� + ���� + ΢���ܿ���
	PID->Integral += PID->Error;                        //�������л���
	if(PID->Integral > PID->Irang)                      //�����޷�
		PID->Integral = PID->Irang;
	if(PID->Integral < -PID->Irang)                     //�����޷�
		PID->Integral = -PID->Irang;                    
	PID->PreError = PID->Error ;                            //ǰһ�����ֵ
}

/*****************************************************************************
* ��  ����void PidParameter_init(void)
* ��  �ܣ���ʼ��PID�ṹ�����һЩ��Աֵ
* ��  ������
* ����ֵ���� 
* ��  ע: ����PID�������Ƕ�������FLASH�У����Դ˺�����ʼ��ʱ���ó�ʼ����Щ������
*         ����Flash�еĲ����п�����Ϊ����������������Flash��ȡ����ʧ�ܣ����
*         ��ʼ��ΪĬ�ϲ�����
*****************************************************************************/
void PidParameter_init(void)
{
	//ROLL��
	PID_ROL_Rate.Ilimit_flag = 1; //Roll����ٶȻ��ֵķ����־
	PID_ROL_Rate.Ilimit = 150;    //Roll����ٶȻ��ַ�Χ
	PID_ROL_Rate.Irang = 1200;    //Roll����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
	PID_ROL_Angle.Ilimit_flag= 1; //Roll��ǶȻ��ֵķ����־
	PID_ROL_Angle.Ilimit = 35;    //Roll��ǶȻ��ַ�Χ
	PID_ROL_Angle.Irang = 200;    //Roll��ǶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�

	//PITCH��
	PID_PIT_Rate.Ilimit_flag = 1; //Pitch����ٶȻ��ֵķ����־
	PID_PIT_Rate.Ilimit = 150;    //Pitch����ٶȻ��ַ�Χ
	PID_PIT_Rate.Irang = 1200;    //Pitch����ٶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
	PID_PIT_Angle.Ilimit_flag = 1;//Roll��ǶȻ��ֵķ����־
	PID_PIT_Angle.Ilimit = 35;    //Roll��ǶȻ��ַ�Χ
	PID_PIT_Angle.Irang = 200;    //Roll��ǶȻ����޷��ȣ����ڵ��������ޣ����Ի������Ҳ�����޵ģ�
	//�߶Ȼ�
	PID_ALT_Rate.Ilimit_flag = 1;
	PID_ALT_Rate.Ilimit = 5;
	PID_ALT_Rate.Irang = 20;
	//���껷
	pid_n.dt=1;
	pid_n.error=0;
	pid_n.iLimit=0;
	pid_n.iLimitLow=0;
	pid_n.integ=0;
	pid_n.kd=0;//
	pid_n.ki=0;
	pid_n.kp=2.5;//
	pid_n.prevError=0;
	//ǰ���ǶȻ�
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
//--------ǰ���Ƕ�PID�ṹ--------------//
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



