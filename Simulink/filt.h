#ifndef __FILT_H__
#define __FILT_H__

#include "stdint.h"

typedef struct
{
    float dat[20];
    uint8_t length;
    uint8_t p;
}FIFO_typedef;

extern void FIFO_DataUpdate(FIFO_typedef *FIFO, float New_dat);
extern float FIFO_Get_Ave(FIFO_typedef *FIFO);
extern float FIFO_Get_Dif(FIFO_typedef *FIFO, uint8_t dif);

#endif
