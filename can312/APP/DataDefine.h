#ifndef __DATADEFINE_H__
#define __DATADEFINE_H__
#include "struct-typedef.h"

#define pi 3.1415926535f
#define Encode_per_round 8191
#define qw (uint16_t)0x01
//============================================================================================//
//can_receive.h
typedef struct{
	//��Ϊһ��ID���Դ����ĸ����������������ĸ�����Ľṹ��

	int16_t speed_1;//���Ϊ1��5�ĵ���ٶȣ�������Ӧ���ǵ��������ơ�������Ǵ������ݵ��ٶȣ��Ѿ�����pid������
	fp32 angle_1;//�����������˼�ǵ�ǰ�ĽǶ�
    int16_t target_angle_1;//Ŀ��Ƕ�

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

	int16_t tagert1_angle;//ϵͳ���������ݣ�����
	int16_t tagert2_angle;
	int16_t tagert3_angle;
	int16_t tagert4_angle;
	

}setPoint;

//�����Ĳ���
typedef struct{
	  int16_t mechain_angle;//���Ǳ�����ֵ
	  int16_t ro_speed;
	  int16_t currents;
	  uint8_t temper;
      
	  //�Ƕȼ���
	  int16_t encode;
	  int16_t preencode;
	  int16_t Total_Round;
      int16_t Total_Encoder;

}motor_data;
//===========================================================================================//

//remoter.h
typedef struct{
	
	uint8_t start;
	
	int16_t ch1;//�Һ�
	int16_t ch3;//����
	
	int16_t ch2;//����
	int16_t ch4;//���
	
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
 
 //���
	int16_t mouse_x; 
	int16_t mouse_y; 
	int16_t mouse_z; 
	int8_t left_button_down; 
	int8_t right_button_down; 
	
 //����
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
 

}RemoteCon;//ң����
//================================================================================================//
//PID

//==============================================================================================//
//classic

#define Car_H 20
#define  Car_W 30


#endif