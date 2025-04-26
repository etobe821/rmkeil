#ifndef __REMOTER_H__
#define __REMOTER_H__

#include "struct-typedef.h"
#include "can-receive.h"
#include "classic.h"
#include "DataDefine.h"



void getRemoterMessage();
void decodefunc(RemoteCon *re,uint8_t *data);

//void controlspeed(motor*motorn,RemoteCon*re);


#endif
