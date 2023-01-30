#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <stdint.h>

#define CH_COUNT 6

typedef struct{
    float fdata[CH_COUNT];
    const uint8_t tail[4];
}Frame;

typedef union{
	Frame FrameData;
	uint8_t UartSendData[CH_COUNT*4 + 4];
}Frame_union;

void SendJustFloat(Frame_union* DataUpToPc);

#endif
