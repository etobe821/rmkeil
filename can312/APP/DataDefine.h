#ifndef __DATADEFINE_H__
#define __DATADEFINE_H__
#include "struct-typedef.h"

#define pi 3.1415926535f
#define Encode_per_round 8191
#define qw (uint16_t)0x01
//============================================================================================//
//can_receive.h
typedef struct{
	//因为一个ID可以搭载四个电机，所以这个是四个电机的结构体

	int16_t speed_1;//电调为1或5的电机速度，本质上应该是电流，类推。这个就是传输数据的速度，已经经过pid调整了
	fp32 angle_1;//代码里面的意思是当前的角度
    int16_t target_angle_1;//目标角度

	int16_t speed_2;
	fp32 angle_2;
	int16_t target_angle_2;

	int16_t speed_3;
	fp32 angle_3;
	int16_t target_angle_3;

	int16_t speed_4;
    fp32 angle_4;
	int16_t target_angle_4;
	
}motor;


typedef struct
{
	int16_t tagert1_speed;
	int16_t tagert2_speed;
	int16_t tagert3_speed;
	int16_t tagert4_speed;

	int16_t tagert1_angle;//系统传来的数据，整形
	int16_t tagert2_angle;
	int16_t tagert3_angle;
	int16_t tagert4_angle;
	

}setPoint;

//反馈的参数
typedef struct{
	  int16_t mechain_angle;//就是编码器值
	  int16_t ro_speed;
	  int16_t currents;
	  uint8_t temper;
      
	  //角度计算
	  int16_t encode;
	  int16_t preencode;
	  int16_t Total_Round;
      int16_t Total_Encoder;

}motor_data;
//===========================================================================================//

//remoter.h
typedef struct{
	
	uint8_t start;
	
	int16_t ch1;//右横
	int16_t ch3;//右竖
	
	int16_t ch2;//左竖
	int16_t ch4;//左横
	
	int16_t SA;
	int16_t SB;
	int16_t SC;
	int16_t SD;
	
	int16_t SE;
	int16_t SF;
	int16_t SG;
	int16_t SH;
	
	int16_t LD;
	int16_t RD;
	int16_t LS;
	int16_t RS;
 
 //鼠标
	int16_t mouse_x; 
	int16_t mouse_y; 
	int16_t mouse_z; 
	int8_t left_button_down; 
	int8_t right_button_down; 
	
 //键盘
	uint16_t w;
	uint16_t s;
	uint16_t a;
	uint16_t d;
	uint16_t shift;
	uint16_t ctrl;
	uint16_t q;
	uint16_t e;
	uint16_t r;
	uint16_t f;
	uint16_t g;
	uint16_t z;
	uint16_t x;
	uint16_t c;
	uint16_t v;
	uint16_t b;
 
	uint16_t reserved; 
 

}RemoteCon;//遥控器
//================================================================================================//
//PID

//==============================================================================================//
//classic

#define Car_H 20
#define  Car_W 30


#endif