#include "classic.h"
#include "main.h"

setPoint spd;//目标速度
extern RemoteCon re1;//遥控器
motor classis_motor;//底盘电机速度
extern motor_data temp[16];//电机反馈参数

//kp 10  ki  3 错的
//初始化pid
#define INITIALIZE_PIDCONTROLLER \
    { \
        .Kp = 10.0f, \
        .Ki = 0.1f, \
        .Kd = 0.0f, \
        .error[0] = 0, \
		.error[1] = 0, \
		.error[2] = 0, \
        .errorI = 0.0f, \
        .errorD = 0.0f, \
        .pidOutput = 0.0f, \
		.pid_Imax = 2000.0f,\
	    .pid_Imin = -2000.0f\
    }


PidController classis_motor_4=INITIALIZE_PIDCONTROLLER;
PidController classis_motor_1=INITIALIZE_PIDCONTROLLER;
PidController classis_motor_2=INITIALIZE_PIDCONTROLLER;
PidController classis_motor_3=INITIALIZE_PIDCONTROLLER;

int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}
		
void wheelSpeedSolute(setPoint *spd,RemoteCon*re){	
	spd->tagert4_speed=(short) (-(re->ch3)-(-(re->ch1))-((re->ch2)*(Car_H/2+Car_W/2)));
//	spd->tagert1_speed=(short) (-(re->ch3)-(-(re->ch1))-((re->ch2)*(Car_H/2+Car_W/2)));
//  spd->tagert1_speed = (short)map(1000, 364.0, 1684.0, 0.0, 2000.0);
	spd->tagert2_speed=(short) ((re->ch3)-((re->ch1))+(-(re->ch2)*(Car_H/2+Car_W/2)));
  spd->tagert3_speed=(short) ((re->ch3)-((re->ch1))+(-(re->ch2)*(Car_H/2+Car_W/2)));
}

// 整车移动量转换为单轮速度  x:前+后-  y:左+右-  z:逆+顺-
//ch1左负右正加负号
//左上到右上,外八，俯视图
//v1=vx-vy-vz
//v2=vx+vy-vz
//v3=vx-vy+vz
//v4=vx+vy+vz

//因为电机方向，正负号看情况调整

void Pid_classis_speed_calculate(void)
{
	classis_motor.speed_4=(int16_t)PidCalculate(&classis_motor_4,(fp32)temp[3].ro_speed,spd.tagert4_speed,position);
    classis_motor.speed_1=(int16_t)PidCalculate(&classis_motor_1,(fp32)temp[0].ro_speed,spd.tagert1_speed,position);
	classis_motor.speed_2=(int16_t)PidCalculate(&classis_motor_2,(fp32)temp[1].ro_speed,spd.tagert2_speed,position);
	classis_motor.speed_3=(int16_t)PidCalculate(&classis_motor_3,(fp32)temp[2].ro_speed,spd.tagert3_speed,position);

}

fp32 classis_encode_calculate(motor_data *motordata)
{
	int16_t delta_encoder;
	motordata->preencode=motordata->encode;
	delta_encoder=motordata->encode-motordata->preencode;

	if (delta_encoder < -4096)
    {
        //正方向转过了一圈
		motordata->Total_Round+=1;
    }
    else if (delta_encoder > 4096)
    {
        //反方向转过了一圈
		motordata->Total_Round-=1;
        
    }
	motordata->Total_Encoder = motordata->Total_Round * Encode_per_round + motordata->mechain_angle;
    return (fp32)(motordata->Total_Encoder*360/Encode_per_round);//返回角度
}

void classis_angle_calculate(void)
{
	classis_motor.angle_4=classis_encode_calculate(&temp[3]);//now angle
	classis_motor.angle_1=classis_encode_calculate(&temp[0]);
	classis_motor.angle_2=classis_encode_calculate(&temp[1]);
	classis_motor.angle_3=classis_encode_calculate(&temp[2]);
	
}

// void target_angle_calculate()
// {}???

void Pid_classis_angle_calculate(void)
{
	classis_motor.target_angle_4=(int16_t)PidCalculate(&classis_motor_4,(fp32)classis_motor.angle_4,spd.tagert4_angle,position);
	classis_motor.target_angle_1=(int16_t)PidCalculate(&classis_motor_1,(fp32)classis_motor.angle_1,spd.tagert1_angle,position);
	classis_motor.target_angle_2=(int16_t)PidCalculate(&classis_motor_2,(fp32)classis_motor.angle_2,spd.tagert2_angle,position);
	classis_motor.target_angle_3=(int16_t)PidCalculate(&classis_motor_3,(fp32)classis_motor.angle_3,spd.tagert3_angle,position);

}

void classistask(void)
{

	while(1)
	{
		wheelSpeedSolute(&spd,&re1);
		Pid_classis_speed_calculate();
		classis_angle_calculate();
		speed_200(&classis_motor);
	    HAL_Delay(2);
	   
	}


}

//PidController pid[4];
//void test(void)
//{
//	for (int i=0;i<=3;i++)
//	{
//		speed[i]=PidCalculate(&pid[i],&temp[i],&target[i]);
//	}

//}

//==================================================================================================================================//

//void PidDataCalculate(motor*motorn,motor_data temp[],setspeed *spd,PidController pid[])
//{
//	pid[3].target=spd->tagert4;
//  pidData(&temp[3],pid,3);
//	motorn->speed_4=(short) (pid[3].pidOutput);
//	
//	pid[0].target=spd->tagert1;
//  pidData(&temp[0],pid,0);
//	motorn->speed_1=pid[0].pidOutput;
//	
//	pid[1].target=spd->tagert2;
//  pidData(&temp[1],pid,1);
//	motorn->speed_2=pid[1].pidOutput;
//	
//	pid[2].target=spd->tagert3;
//  pidData(&temp[2],pid,2);
//  motorn->speed_3=pid[2].pidOutput;
//}

//void controlspeed(motor*motorn,motor_data temp[],setspeed *spd,PidController pid[])
//{
//	wheelSpeedSolute(motorn,re);
//	PIDcalculate(motorn,&temp,spd,&pid);
//	speed_200(motorn);
//}