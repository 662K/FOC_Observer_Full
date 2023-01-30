#include "filt.h"

/********************************INT16***********************************/
void FIFO_DataUpdate_int16(FIFO_int16 *FIFO, int16_t New_dat){
	FIFO->dat[FIFO->p] = New_dat;
	FIFO->p++;
	if(FIFO->p >= FIFO->length)
		FIFO->p -= FIFO->length;
}

int16_t FIFO_Get_Ave_int16(FIFO_int16 *FIFO){
	int32_t sum = 0;
	
	for(uint16_t i = 0; i < FIFO->length; ++i){
		sum += FIFO->dat[i];
	}
	
	return sum/FIFO->length;
}

int16_t FIFO_Get_Dif_int16(FIFO_int16 *FIFO, uint16_t dif){
	int16_t p,p_last;

	(FIFO->p == 0) ? (p = FIFO->length - 1) : (p = FIFO->p - 1);

	p_last = p - dif;
	if (p_last < 0) p_last += FIFO->length;

	return FIFO->dat[p] - FIFO->dat[p_last];
}

/********************************FLOAT***********************************/
void FIFO_DataUpdate_float(FIFO_float *FIFO, float New_dat){
	FIFO->dat[FIFO->p] = New_dat;
	FIFO->p++;
	if(FIFO->p >= FIFO->length)
		FIFO->p -= FIFO->length;
}

float FIFO_Get_Ave_float(FIFO_float *FIFO){
	float sum = 0;
	
	for(uint8_t i = 0; i < FIFO->length; ++i){
		sum += FIFO->dat[i];
	}
	
	return sum/FIFO->length;
}

float FIFO_Get_Dif_float(FIFO_float *FIFO, uint8_t dif){
	int8_t p, p_last;

	(FIFO->p == 0) ? (p = FIFO->length - 1) : (p = FIFO->p - 1);

	p_last = p - dif;
	if (p_last < 0) p_last += FIFO->length;

	return FIFO->dat[p] - FIFO->dat[p_last];
}
