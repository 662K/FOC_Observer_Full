#ifndef __FOCSUB_H__
#define __FOCSUB_H__

#include "main.h"

void Cordic(float ThetaE, float* SinTheta, float* CosTheta);
void InvPark(float Ud, float Uq, float SinTheta, float CosTheta, float* Ux, float* Uy);
void InvClarke(float Ux, float Uy, float* U1, float* U2, float* U3);
void GetSector(float U1, float U2, float U3, uint8_t* Sector);
void GetCCR(float U1, float U2, float U3, uint8_t Sector, float Uac, float* CCRa, float* CCRb, float* CCRc);
void Clarke(float Ia, float Ic, float* Ix, float* Iy);
void Park(float Ix, float Iy, float SinTheta, float CosTheta, float* Id, float* Iq);
void Spd_Timer(int8_t* Spd_Tick);
void GetSpd(uint32_t Theta, uint32_t* Theta_Pre, float* Speed, float SpdFs, int8_t Spd_Tick, int8_t* Start_Flag);

#endif
