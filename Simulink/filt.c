#include "filt.h"

void FIFO_DataUpdate(FIFO_typedef *FIFO, float New_dat){
    FIFO->dat[FIFO->p] = New_dat;
    FIFO->p++;
    if(FIFO->p >= FIFO->length)
        FIFO->p -= FIFO->length;
}

float FIFO_Get_Ave(FIFO_typedef *FIFO){
    float sum = 0;

    for(uint8_t i = 0; i < FIFO->length; ++i){
        sum += FIFO->dat[i];
    }

    return sum/FIFO->length;
}

float FIFO_Get_Dif(FIFO_typedef *FIFO, uint8_t dif){
    int8_t p, p_last;

    (FIFO->p == 0) ? (p = FIFO->length - 1) : (p = FIFO->p - 1);

    p_last = p - dif;
    if (p_last < 0) p_last += FIFO->length;

    return FIFO->dat[p] - FIFO->dat[p_last];
}
