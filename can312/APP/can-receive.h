#ifndef __CAN_RECVIVE_H__
#define __CAN_RECVIVE_H__
#include "struct-typedef.h"

#include "DataDefine.h"


void can_filter_init(void);

//speed
//void speed(int a,int b,int c,int d);
void speed_200(motor *motor_1);
void speed_1ff(motor *motor_3,motor*motor_4);
//void speed(motor *motor_1,motor*motor_2,motor *motor_3,motor*motor_4);


//receive

void getmessage(motor_data temp[],uint8_t *RxData,int i);

#endif
