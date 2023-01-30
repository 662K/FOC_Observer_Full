#ifndef __MAIN_H__
#define __MAIN_H__

#include <math.h>
#include <stdint.h>
#include "filt.h"

typedef struct{
    float Kp;
    float Ki;
    float Max;
    float up;
    float ui;
    float Error;
    float Out_temp;
}PI_str;

typedef struct{
    uint8_t Spd_Tick;
    float Theta_Pre;
    
    float Spd;
    float Id_Target;

    float Id;
    float Iq;

    float CurTs;
    float SpdTs;
    float CurFs;
    float SpdFs;

    float Ud;
    float Uq;

    uint8_t Mode;
    int Start_Flag;
}ControlCommand_str;

typedef struct{
    float Ls;
    float Ld;
    float Lq;
    float Rs;
    float Kt;
    float J;
    float Flux;
    uint8_t Np;
}MotorParameter_str;

typedef struct{
    float Theta;
    float Spd;

    float Udc;

    float SinTheta;
    float CosTheta; 

    float Ux;       
    float Uy;
    float Ux_temp;       
    float Uy_temp;

    float Ux_Delay1;
    float Uy_Delay1;
    float Ux_Delay2;
    float Uy_Delay2;

    float U1;    
    float U2;   
    float U3;       

    uint8_t Sector;

    float CCRa;
    float CCRb; 
    float CCRc;

    float Ia;
    float Ic;

    float Ix;       
    float Iy;

    float Ix_Delay1;
    float Iy_Delay1;
    float Ix_Delay2;
    float Iy_Delay2;

    float Id;
    float Iq;

    float Ud;
    float Uq;

    float Ex;
    float Ey;

    float ThetaE;

    float EMF_Peak;
    float EMF_Rms;
}MotorRealTimeInformation_str;

typedef struct{
    float Te;
    float TL;
    float Acc;
    float Spd;
    float Spd_Temp;
    float Spd_Bef;
    float Spd_Pre;
    float Theta;
    float Theta_Pre;
    PI_str Spd_PI;
}MotorObserver_str;

typedef struct{
    float Vx;
    float Vy;
    float Ex;
    float Ey;
    float Ex_Test;
    float Ey_Test;
    float Ix;
    float Iy;
    float ThetaE;
    float SpdE;
    float EMF_Rms;
    float SinTheta;
    float CosTheta;
    int Qx;
    int Qy;
    int Dir;
    int Dir_status;

    float de;
    PI_str PLL_PI;
    PI_str NPLL_PI;
    float Theta_PLL_wn;
    float Theta_PLL_zeta;
    float Theta_PLL_we;

    float ThetaE_Temp;

    float h1;
    float h2;
    float E1;
    float EMF_LPF_wc;
    float m;
    float Spd_LPF_wc;
    float Switch_Spd;
    float Switch_EMF;

    int status;
}SlidingModeObserver_str;

#define PI acos(-1)
#define TRUE 1
#define FALSE 0

#endif

