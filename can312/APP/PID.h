#ifndef __PID_H__
#define __PID_H__
#include "struct-typedef.h"
#include "remoter.h"
#include "can-receive.h"
#include "DataDefine.h"

typedef enum{
	position,
	delta

}PID_type;

typedef struct{
	
	fp32 Kp;
	fp32 Ki;
	fp32 Kd;
	fp32 error[3];
	fp32 errorI;
	fp32 errorD;
	fp32 pidOutput;
	fp32 pid_Imax;
	fp32 pid_Imin;

}PidController;

fp32 PidCalculate(PidController *pid,fp32 data_now,fp32 target,int judgeType);




//void pidData(motor_data temp[],PidController pid[],int i);




#endif