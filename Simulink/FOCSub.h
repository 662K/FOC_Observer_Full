#ifndef __FOCSUB_H__
#define __FOCSUB_H__

#include "main.h"

extern void Cordic(float ThetaE, float* SinTheta, float* CosTheta);
extern void InvPark(float Ud, float Uq, float SinTheta, float CosTheta, float* Ux, float* Uy);
extern void InvClarke(float Ux, float Uy, float* U1, float* U2, float* U3);
extern uint8_t GetSector(float U1, float U2, float U3);
extern void GetCCR(float U1, float U2, float U3, uint8_t Sector, float Udc, float* CCRa, float* CCRb, float* CCRc);
extern void Clarke(float Ia, float Ic, float* Ix, float* Iy);
extern void Park(float Ix, float Iy, float SinTheta, float CosTheta, float* Id, float* Iq);
extern void Spd_Timer(uint8_t* Spd_Tick);

#endif
