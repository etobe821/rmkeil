#include "remoter.h"
#include "main.h"

extern UART_HandleTypeDef huart1;

uint8_t datasave[32];

RemoteCon re1;

void getRemoterMessage()
{
	HAL_UART_Receive_DMA(&huart1,datasave,32);

}

void decodefunc(RemoteCon *re,uint8_t *data)
{
	if (data[0]!=0x0f)
		return;
	re->start=data[0];
	re->ch1=((uint16_t)data[1]|(uint16_t)data[2]<<8)&0x07ff;//第二个就是移位剩余的
	re->ch1-=1024;
	re->ch2=((uint16_t)(data[2])>>3|((uint16_t)data[3])<<5)&0x07ff;//3剩余2
	re->ch2-=1024;
  re->ch3=((uint16_t)(data[3])>>6|((uint16_t)data[4])<<2|((uint16_t)data[5])<<10)&0x07ff;//5 s 7
	re->ch3-=1024;
	re->ch4=((uint16_t)(data[5])>>1|((uint16_t)data[6])<<7)&0x07ff;//6 s 4
	re->ch4-=1024;
    
	re->SA = ((uint16_t)((data[6] & 0xf0) >> 4)) | (((uint16_t)(data[7] & 0x7f)) << 4);
  re->SB = ((uint16_t)((data[7] & 0x80) >> 7)) | (((uint16_t)data[8]) << 1) | (((uint16_t)(data[9] & 0x03)) << 9);
  re->SC = ((uint16_t)((data[9] & 0xfc) >> 2)) | (((uint16_t)(data[10] & 0x1f)) << 6);
  re->SD = ((uint16_t)((data[10] & 0xe0) >> 5)) | (((uint16_t)(data[11])) << 3);
  re->SE = ((uint16_t)data[12]) | (((uint16_t)(data[13] & 0x07)) << 8);
  re->SF = ((uint16_t)((data[13] & 0xf8) >> 3)) | (((uint16_t)(data[14] & 0x3f)) << 5);
  re->SG = ((uint16_t)((data[14] & 0xc0) >> 6)) | (((uint16_t)data[15]) << 2) | (((uint16_t)(data[16] & 0x01)) << 10);
  re->SH = ((uint16_t)((data[16] & 0xfe) >> 1)) | (((uint16_t)(data[17] & 0x0f)) << 7);
	
	re->LD = ((uint16_t)((data[17] & 0xf0) >> 4)) | (((uint16_t)(data[18] & 0x7f)) << 4);
  re->RD = ((uint16_t)((data[18] & 0x80) >> 7)) | (((uint16_t)data[19]) << 1) | (((uint16_t)(data[20] & 0x03)) << 9);
  re->LS = ((uint16_t)((data[20] & 0xfc) >> 2)) | (((uint16_t)(data[21] & 0x1f)) << 6);
  re->RS = ((uint16_t)((data[21] & 0xe0) >> 5)) | (((uint16_t)data[22]) << 3);
	
	//键鼠解码
	re->mouse_x=((int16_t)data[23]) | ((int16_t)data[24] << 8);
	re->mouse_y=((int16_t)data[25]) | ((int16_t)data[26] << 8);
	re->mouse_z=((int16_t)data[27]) | ((int16_t)data[28] << 8);
	re->left_button_down=(int16_t)data[29];
	re->right_button_down=(int16_t)data[30];
	
	re->w=(uint16_t)data[31]&qw;
	re->s=((uint16_t)data[31]&(qw<<1))>>1;
	re->a=((uint16_t)data[31]&(qw<<2))>>2;
	re->d=((uint16_t)data[31]&(qw<<3))>>3;
	re->shift=((uint16_t)data[31]&(qw<<4))>>4;
	re->ctrl=((uint16_t)data[31]&(qw<<5))>>5;
	re->q=((uint16_t)data[31]&(qw<<6))>>6;
	re->e=((uint16_t)data[31]&(qw<<7))>>7;
	
	re->r=((uint16_t)data[32]&(qw<<0))>>0;
	re->f=((uint16_t)data[32]&(qw<<1))>>1;
	re->g=((uint16_t)data[32]&(qw<<2))>>2;
	re->z=((uint16_t)data[32]&(qw<<3))>>3;
	re->x=((uint16_t)data[32]&(qw<<4))>>4;
	re->c=((uint16_t)data[32]&(qw<<5))>>5;
	re->v=((uint16_t)data[32]&(qw<<6))>>6;
	re->b=((uint16_t)data[32]&(qw<<7))>>7;
	
}
		
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	decodefunc(&re1,datasave);

}


