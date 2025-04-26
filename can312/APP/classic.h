#ifndef __CLASSIC_H__
#define __CLASSIC_H__
#include "struct-typedef.h"
#include "DataDefine.h"
#include "PID.h"

void wheelSpeedSolute(setPoint *spd,RemoteCon*re);

//void PidDataCalculate(motor*motorn,motor_data temp[],setspeed *spd,PidController pid[]);

void classistask(void);

//void controlspeed(motor*motorn,motor_data temp[],setspeed *spd,PidController pid[]);

void Pid_classis_speed_calculate(void);

int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);

fp32 classis_encode_calculate(motor_data *motordata);
void classis_angle_calculate(void);

void Pid_classis_angle_calculate(void);

#endif
