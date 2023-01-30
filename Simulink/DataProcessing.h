#ifndef __DATAPROCESSING_H__
#define __DATAPROCESSING_H__

#include "main.h"
#include "FOCSub.h"
#include "filt.h"

float PID_Control(PI_str* pPI, float Target, float Present);
float PID_Control_Err(PI_str* pPI, float Error);
float PI_Control(PI_str* pPI);
float PIMAX_Control(PI_str* pPI, float Target, float Present, float MaxUp, float MaxDown);
void LPF(float* Uo, float Ui, float Fs, float Fc);

#endif
