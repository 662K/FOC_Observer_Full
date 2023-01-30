#ifndef __DATASAMPLING_H__
#define __DATASAMPLING_H__

#include "main.h"

extern float GetTheta(int32_t Theta);
extern float GetCur(int32_t Cur);
extern float GetThetaE(float ThetaE, uint8_t Np);
extern void GetSpd(float Theta, float* Theta_Pre, uint8_t Spd_Tick, float* Speed, float SpdTs, int* Start_Flag);
extern void CtrlComFilter(float *Target, float CtrlCom, float TickAdd);

#endif
