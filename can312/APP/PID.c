#include "PID.h"

fp32 limit_PIDI(fp32 value,fp32 max,fp32 min)//»ý·ÖÏÞ·ù
{
	if(value>=max){value=max;}
	else if (value<=min){value=min;}
	return value;

}

fp32 PidCalculate(PidController *pid,fp32 data_now,fp32 target,int judgeType)
{
	pid->error[2]=pid->error[1];
	pid->error[1]=pid->error[0];
	pid->error[0]=-data_now+target;
	
	if(judgeType==0)//position
	{
		
			pid->errorI+=pid->error[0];
		  pid->errorI=limit_PIDI(pid->errorI,pid->pid_Imax,pid->pid_Imin);
			pid->errorD=-pid->error[0]+pid->error[1];
			pid->pidOutput=pid->Kp*pid->error[0]+pid->errorI*pid->Ki+pid->errorD*pid->Kd;
	}
	else if (judgeType==1)//delta
	{
		  
			pid->errorD=pid->error[0]-2*pid->error[1]+pid->error[2];
			pid->pidOutput=pid->Kp*(pid->error[0]-pid->error[1])+pid->error[0]*pid->Ki+pid->errorD*pid->Kd;
	}
	
	else
		pid->pidOutput=0.0f;
	
	return pid->pidOutput;
	
}



//void pidData(motor_data temp[],PidController pid[],int i)
//{
//	pid[i].point_now=temp[i].ro_speed;
//	pid[i].error=(fp32)((pid[i].point_now)-(pid[i].target));
//	pid[i].errorI=(pid[i].previoua_error)+(pid[i].error);
//	pid[i].errorD=(pid[i].error)-(pid[i].previoua_error);
//	pid[i].previoua_error=pid[i].error;
//	pid[i].pidOutput=pid[i].Kp*pid[i].error+pid[i].Ki*pid[i].errorI+pid[i].Kd*pid[i].errorD;
//	
//}


//void PIDcalculate(PidController *pid[],int i)
//{
//	
//	
//	pid->error[1]=(fp32)((pid->point_now[1])-(pid->target[1]));
//	pid->errorI[1]=(pid->previoua_error[1])+(pid->error[1]);
//	pid->errorD[1]=(pid->error[1])-(pid->previoua_error[1]);
//	pid->previoua_error[1]=pid->error[1];
//	pid->pidOutput[1]=pid->Kp*pid->error[1]+pid->Ki*pid->errorI[1]+pid->Kd*pid->errorD[1];
//	
//	pid->error[2]=(fp32)((pid->point_now[2])-(pid->target[2]));
//	pid->errorI[2]=(pid->previoua_error[2])+(pid->error[2]);
//	pid->errorD[2]=(pid->error[2])-(pid->previoua_error[2]);
//	pid->previoua_error[2]=pid->error[2];
//	pid->pidOutput[2]=pid->Kp*pid->error[2]+pid->Ki*pid->errorI[2]+pid->Kd*pid->errorD[2];
//	
//	pid->error[3]=(fp32)((pid->point_now[3])-(pid->target[3]));
//	pid->errorI[3]=(pid->previoua_error[3])+(pid->error[3]);
//	pid->errorD[3]=(pid->error[3])-(pid->previoua_error[3]);
//	pid->previoua_error[3]=pid->error[3];
//	pid->pidOutput[3]=pid->Kp*pid->error[3]+pid->Ki*pid->errorI[3]+pid->Kd*pid->errorD[3];
//	
//	
//}