#include "FOCSub.h"

//float cosOutput, sinOutput;
void Cordic(float ThetaE, float* SinTheta, float* CosTheta){
    LL_CORDIC_WriteData(CORDIC, ThetaE / (2 * PI) * 0xFFFFFFFF);
    *CosTheta = 1.0f * ((int32_t)LL_CORDIC_ReadData(CORDIC)) / 0x7FFFFFFF;
    *SinTheta = 1.0f * ((int32_t)LL_CORDIC_ReadData(CORDIC)) / 0x7FFFFFFF;
}

void InvPark(float Ud, float Uq, float SinTheta, float CosTheta, float* Ux, float* Uy){
    *Ux = CosTheta * Ud - SinTheta * Uq;
    *Uy = SinTheta * Ud + CosTheta * Uq;
}

void InvClarke(float Ux, float Uy, float* U1, float* U2, float* U3){
    *U1 =  Uy;
    *U2 =  Ux * 0.866f - Uy * 0.5f;
    *U3 = -Ux * 0.866f - Uy * 0.5f;
}

void GetSector(float U1, float U2, float U3, uint8_t* Sector){
    switch (((U3 > 0) << 2) | ((U2 > 0) << 1) | ((U1 > 0) << 0)){
    case 3: *Sector = 1; break;
    case 1: *Sector = 2; break;
    case 5: *Sector = 3; break;
    case 4: *Sector = 4; break;
    case 6: *Sector = 5; break;
    case 2: *Sector = 6; break;}
}

void GetCCR(float U1, float U2, float U3, uint8_t Sector, float Uac, float* CCRa, float* CCRb, float* CCRc){
    static float Tx, Ty;
    static float Ta, Tb, Tc;
    switch(Sector){
    case 1: Tx =  U2 / Uac; Ty =  U1 / Uac; break;
    case 2: Tx = -U2 / Uac; Ty = -U3 / Uac; break;
    case 3: Tx =  U1 / Uac; Ty =  U3 / Uac; break;
    case 4: Tx = -U1 / Uac; Ty = -U2 / Uac; break;
    case 5: Tx =  U3 / Uac; Ty =  U2 / Uac; break;
    case 6: Tx = -U3 / Uac; Ty = -U1 / Uac; break;}

    Ta = (1 - Tx - Ty) / 2;
    Tb = (1 + Tx - Ty) / 2;
    Tc = (1 + Tx + Ty) / 2;

    switch(Sector){
    case 1: *CCRa = Ta;   *CCRb = Tb;   *CCRc = Tc;   break;
    case 2: *CCRa = Tb;   *CCRb = Ta;   *CCRc = Tc;   break;
    case 3: *CCRa = Tc;   *CCRb = Ta;   *CCRc = Tb;   break;
    case 4: *CCRa = Tc;   *CCRb = Tb;   *CCRc = Ta;   break;
    case 5: *CCRa = Tb;   *CCRb = Tc;   *CCRc = Ta;   break;
    case 6: *CCRa = Ta;   *CCRb = Tc;   *CCRc = Tb;   break;
    }
}

void Clarke(float Ia, float Ic, float* Ix, float* Iy){
    *Ix = Ia;
    *Iy = -((Ia + Ic * 2.0f)*0.57735f);
}

void Park(float Ix, float Iy, float SinTheta, float CosTheta, float* Id, float* Iq){
    *Id = SinTheta * Iy + CosTheta * Ix;
    *Iq = CosTheta * Iy - SinTheta * Ix;
}

void Spd_Timer(int8_t* Spd_Tick){
    if(*Spd_Tick < 9)
        *Spd_Tick += 1;
    else
        *Spd_Tick = 0;
}

void GetSpd(uint32_t Theta, uint32_t* Theta_Pre, float* Speed, float SpdFs, int8_t Spd_Tick, int8_t* Start_Flag){
    static int32_t delta_Theta;
    
    if(Spd_Tick == 0){
        if(*Start_Flag != 1){
            *Start_Flag = 1;
            *Theta_Pre = Theta;
        }
        else{
            delta_Theta = (Theta - *Theta_Pre) << 15;
            delta_Theta = delta_Theta >> 15;
            
            *Speed = (2 * PI) * delta_Theta / (1 << 17) * SpdFs;
            *Theta_Pre = Theta;
        }
    }
}
