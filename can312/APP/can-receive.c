#include "can-receive.h"
#include  "main.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;


extern UART_HandleTypeDef huart1;

CAN_TxHeaderTypeDef header_1,header_2;	



//speed里面的数组
uint8_t data_1[8];
uint8_t data_2[8];
uint8_t data_3[8];
uint8_t data_4[8];

void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
	
    can_filter_st.FilterMaskIdHigh = 0x0000; 
    can_filter_st.FilterMaskIdLow = 0x0000;
	
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
		can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);



}



		


void speed_200(motor *motor_1)
{
	uint32_t mail_1,mail_2;
	header_1.DLC=0x08;
	header_1.IDE=CAN_ID_STD;
	header_1.RTR=CAN_RTR_DATA;
	header_1.StdId=0x200;
	
	data_1[0]=motor_1->speed_1>>8;
	data_1[1]=motor_1->speed_1;
	data_1[2]=motor_1->speed_2>>8;
	data_1[3]=motor_1->speed_2;
	data_1[4]=motor_1->speed_3>>8;
	data_1[5]=motor_1->speed_3;
	data_1[6]=motor_1->speed_4>>8;
	data_1[7]=motor_1->speed_4;
	HAL_CAN_AddTxMessage(&hcan1,&header_1,data_1,&mail_1);
	
//	data_2[0]=motor_2->speed_1>>8;
//	data_2[1]=motor_2->speed_1;
//	data_2[2]=motor_2->speed_2>>8;
//	data_2[3]=motor_2->speed_2;
//	data_2[4]=motor_2->speed_3>>8;
//	data_2[5]=motor_2->speed_3;
//	data_2[6]=motor_2->speed_4>>8;
//	data_2[7]=motor_2->speed_4;
//	HAL_CAN_AddTxMessage(&hcan2,&header_1,data_2,&mail_2);
}

void speed_1ff(motor *motor_3,motor*motor_4)
{
	uint32_t mail_1,mail_2;
	header_2.DLC=0x08;
	header_2.IDE=CAN_ID_STD;
	header_2.RTR=CAN_RTR_DATA;
	header_2.StdId=0x1FF;
	
	data_3[0]=motor_3->speed_1>>8;
	data_3[1]=motor_3->speed_1;
	data_3[2]=motor_3->speed_2>>8;
	data_3[3]=motor_3->speed_2;
	data_3[4]=motor_3->speed_3>>8;
	data_3[5]=motor_3->speed_3;
	data_3[6]=motor_3->speed_4>>8;
	data_3[7]=motor_3->speed_4;
	HAL_CAN_AddTxMessage(&hcan1,&header_2,data_3,&mail_1);
	
	data_4[0]=motor_4->speed_1>>8;
	data_4[1]=motor_4->speed_1;
	data_4[2]=motor_4->speed_2>>8;
	data_4[3]=motor_4->speed_2;
	data_4[4]=motor_4->speed_3>>8;
	data_4[5]=motor_4->speed_3;
	data_4[6]=motor_4->speed_4>>8;
	data_4[7]=motor_4->speed_4;
	HAL_CAN_AddTxMessage(&hcan2,&header_2,data_4,&mail_2);

}


//void speed(motor *motor_1,motor*motor_2,motor *motor_3,motor*motor_4)
//{
//	speed_200(motor_1,motor_2);
//	speed_1ff(motor_3,motor_4);

//}


//反馈数据
motor_data temp[16];

void getmessage(motor_data temp[],uint8_t *RxData,int i)
{
		
		temp[i].mechain_angle=(uint16_t)((RxData)[0]<<8|(RxData)[1]);
	  temp[i].ro_speed=(uint16_t)((RxData)[2]<<8|(RxData)[3]);
		temp[i].currents=(uint16_t)((RxData)[4]<<8|(RxData)[5]);
		temp[i].temper=(RxData[6]);
	
	//(angle*360)/8191
	 
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
			 
				CAN_RxHeaderTypeDef RxHeader_1,RxHeader_2;
				uint8_t RxData[8];
			  HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxHeader_1,RxData);
			  switch(RxHeader_1.StdId){
					case 0x201:
						getmessage(temp,RxData,0);
					  break;
					case 0x202:
						getmessage(temp,RxData,1);
					  break;
					case 0x203:
						getmessage(temp,RxData,2);
					  break;
					case 0x204:
						getmessage(temp,RxData,3);
					  break;
					case 0x205:
						getmessage(temp,RxData,4);
					  break;
					case 0x206:
						getmessage(temp,RxData,5);
					  break;
					case 0x207:
						getmessage(temp,RxData,6);
					  break;
					case 0x208:
						getmessage(temp,RxData,7);
					  break;}
				HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxHeader_2,RxData);
					switch(RxHeader_2.StdId){
					case 0x201:
						getmessage(temp,RxData,8);
					  break;
					case 0x202:
						getmessage(temp,RxData,9);
					  break;
					case 0x203:
						getmessage(temp,RxData,10);
					  break;
					case 0x204:
						getmessage(temp,RxData,11);
					  break;
					case 0x205:
						getmessage(temp,RxData,12);
					  break;
					case 0x206:
						getmessage(temp,RxData,13);
					  break;
					case 0x207:
						getmessage(temp,RxData,14);
					  break;
					case 0x208:
						getmessage(temp,RxData,15);
					  break;
				
				
		
       					
				
			}

}



