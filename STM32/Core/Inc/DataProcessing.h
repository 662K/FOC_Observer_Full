#ifndef __DATAPROCESSING_H__
#define __DATAPROCESSING_H__

#include "main.h"
#include "FOCSub.h"

void CtrlComFilter(float *Target, float CtrlCom, float TickAdd);
void LPF(float* Uo, float Ui, float Fs, float Wc);
float PI_Control(PI_str* pPI);
float sawtooth(float angle, float width);

#endif
