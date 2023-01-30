#ifndef __FILT_H__
#define __FILT_H__

#include "stdint.h"

typedef struct
{
	int16_t dat[501];
	uint16_t length;
	uint16_t p;
}FIFO_int16;

void FIFO_DataUpdate_int16(FIFO_int16 *FIFO, int16_t New_dat);
int16_t FIFO_Get_Ave_int16(FIFO_int16 *FIFO);
int16_t FIFO_Get_Dif_int16(FIFO_int16 *FIFO, uint16_t dif);

typedef struct
{
	float dat[20];
	uint8_t length;
	uint8_t p;
}FIFO_float;

void FIFO_DataUpdate_float(FIFO_float *FIFO, float New_dat);
float FIFO_Get_Ave_float(FIFO_float *FIFO);
float FIFO_Get_Dif_float(FIFO_float *FIFO, uint8_t dif);

#endif
