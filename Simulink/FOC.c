#include "FOC.h"
#include "math.h"
#include "FOCSub.h"
#include "DataSampling.h"
#include "DataProcessing.h"

void SpeedLoopOri_Mode0(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlComFilter(&MRT_Inf->Ud, 27, 0.01);
    }

    InvPark(MRT_Inf->Ud, 0, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void CurLoop_Mode1(PI_str* D_PI, PI_str* Q_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    CtrlCom->Id = CtrlCom->Id_Target;
    CtrlCom->Iq = 0;

    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void SpeedLoopOri_Mode2(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

float SMOSwitchFunction0(float Error){
    float SF_Out;

    if(Error > 0)
        SF_Out = 1.0;
    else
        SF_Out = -1.0;

    return SF_Out;
}

float SMOSwitchFunction1(float E, float Error){
    float SF_Out;

    if(Error > E)
        SF_Out = 1.0;
    else if(Error < -E)
        SF_Out = -1.0;
    else
        SF_Out = 1.0 / E * Error;

    return SF_Out;
}

float SMOSwitchFunction2(float m, float Error){
    float peError, neError;

    peError = expf(m*Error);
    neError = expf(-m*Error);

    return ((peError - neError) / (peError + neError));
}

void SMO_Mode0(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction0(SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction0(SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    LPF(&(SMO->Ex), SMO->h1 * SMO->Vx, CtrlCom->CurFs, SMO->EMF_LPF_wc);
    LPF(&(SMO->Ey), SMO->h1 * SMO->Vy, CtrlCom->CurFs, SMO->EMF_LPF_wc);

    SMO->ThetaE = -atan2f(SMO->Ex, SMO->Ey);
    if(SMO->ThetaE < 0)
        SMO->ThetaE += 2 * PI;

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void SMO_Mode1(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    LPF(&(SMO->Ex), SMO->h1 * SMO->Vx, CtrlCom->CurFs, SMO->EMF_LPF_wc);
    LPF(&(SMO->Ey), SMO->h1 * SMO->Vy, CtrlCom->CurFs, SMO->EMF_LPF_wc);

    SMO->ThetaE = -atan2f(SMO->Ex, SMO->Ey);
    if(SMO->ThetaE < 0)
        SMO->ThetaE += 2 * PI;

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void SMO_Mode2(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction2(SMO->m, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction2(SMO->m, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    LPF(&(SMO->Ex), SMO->h1 * SMO->Vx, CtrlCom->CurFs, SMO->EMF_LPF_wc);
    LPF(&(SMO->Ey), SMO->h1 * SMO->Vy, CtrlCom->CurFs, SMO->EMF_LPF_wc);

    SMO->ThetaE = -atan2f(SMO->Ex, SMO->Ey);
    if(SMO->ThetaE < 0)
        SMO->ThetaE += 2 * PI;

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void HSMO_Mode(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    SMO->Ex = SMO->Ex + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey + SMO->h2 * SMO->Vx / MotorParameter->Ls);
    SMO->Ey = SMO->Ey + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex + SMO->h2 * SMO->Vy / MotorParameter->Ls);

    SMO->EMF_Rms = sqrtf(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey);

    switch(SMO->status){
        case 0:
            if(SMO->EMF_Rms > SMO->Switch_EMF * 0.5f){
                SMO->status = 1;
            }
            else{
                SMO->status = 0;
            }
            break;
        case 1:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else if(SMO->EMF_Rms > SMO->Switch_EMF){
                SMO->status = 2;
            }
            else{
                SMO->status = 1;
            }
            break;
        case 2:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else{
                SMO->status = 2;
            }
            break;
    }

    Cordic(SMO->ThetaE, &SMO->SinTheta, &SMO->CosTheta);

    float SMO_SpdE_Temp = 0;
    float SMO_ThetaE_temp = 0;

    switch(SMO->status){
        case 0:
            SMO->ThetaE = MRT_Inf->ThetaE;
            SMO->SpdE = MRT_Inf->Spd * MotorParameter->Np;
            break;
        case 1:
        case 2:
            SMO->ThetaE_Temp = SMO->ThetaE;
            SMO->ThetaE = -atan2f(SMO->Ex, SMO->Ey);
            if(SMO->ThetaE < 0)
                SMO->ThetaE += 2 * PI;

            SMO_SpdE_Temp = SMO->ThetaE - SMO->ThetaE_Temp;

            if(SMO_SpdE_Temp < -PI){
                SMO_SpdE_Temp = SMO_SpdE_Temp + 2*PI;
            }
            else if(SMO_SpdE_Temp > PI){
                SMO_SpdE_Temp = SMO_SpdE_Temp - 2*PI;
            }
            else{
                SMO_SpdE_Temp = SMO_SpdE_Temp;
            }

            LPF(&(SMO->SpdE), SMO_SpdE_Temp * CtrlCom->CurFs, CtrlCom->CurFs, SMO->Spd_LPF_wc);
            break;
    }

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void HSMO_Mode2(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    SMO->Ex = SMO->Ex + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey + SMO->h2 * SMO->Vx / MotorParameter->Ls);
    SMO->Ey = SMO->Ey + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex + SMO->h2 * SMO->Vy / MotorParameter->Ls);

    SMO->EMF_Rms = sqrtf(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey);

    switch(SMO->status){
        case 0:
            if(SMO->EMF_Rms > SMO->Switch_EMF * 0.5f){
                SMO->status = 1;
            }
            else{
                SMO->status = 0;
            }
            break;
        case 1:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else if(SMO->EMF_Rms > SMO->Switch_EMF){
                SMO->status = 2;
            }
            else{
                SMO->status = 1;
            }
            break;
        case 2:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else{
                SMO->status = 2;
            }
            break;
    }

    Cordic(SMO->ThetaE, &SMO->SinTheta, &SMO->CosTheta);

    float SMO_SpdE_Temp = 0;
    float SMO_ThetaE_temp = 0;

    switch(SMO->status){
        case 0:
            SMO->ThetaE = MRT_Inf->ThetaE;
            SMO->SpdE = MRT_Inf->Spd * MotorParameter->Np;
            SMO->NPLL_PI.ui = SMO->SpdE;
            break;
        case 1:
            SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->Switch_EMF;
            SMO_SpdE_Temp = PID_Control_Err(&(SMO->NPLL_PI), SMO->de);
            LPF(&(SMO->SpdE), SMO_SpdE_Temp, CtrlCom->CurFs, SMO->Spd_LPF_wc);

            SMO_ThetaE_temp = SMO->ThetaE + SMO_SpdE_Temp * CtrlCom->CurTs;
            if(SMO_ThetaE_temp < 0)
                SMO_ThetaE_temp += 2 * PI;
            SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
            break;
        case 2:
            SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->EMF_Rms;
            SMO_SpdE_Temp = PID_Control_Err(&(SMO->NPLL_PI), SMO->de);
            LPF(&(SMO->SpdE), SMO_SpdE_Temp, CtrlCom->CurFs, SMO->Spd_LPF_wc);

            SMO_ThetaE_temp = SMO->ThetaE + SMO_SpdE_Temp * CtrlCom->CurTs;
            if(SMO_ThetaE_temp < 0)
                SMO_ThetaE_temp += 2 * PI;
            SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
            break;
    }

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void HSMO_Mode3(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    SMO->Ex = SMO->Ex + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey + SMO->h2 * SMO->Vx / MotorParameter->Ls);
    SMO->Ey = SMO->Ey + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex + SMO->h2 * SMO->Vy / MotorParameter->Ls);

    SMO->EMF_Rms = sqrtf(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey);

    switch(SMO->status){
        case 0:
            if(SMO->EMF_Rms > SMO->Switch_EMF * 0.5f){
                SMO->status = 1;
            }
            else{
                SMO->status = 0;
            }
            break;
        case 1:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else if(SMO->EMF_Rms > SMO->Switch_EMF){
                SMO->status = 2;
            }
            else{
                SMO->status = 1;
            }
            break;
        case 2:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else{
                SMO->status = 2;
            }
            break;
    }

    Cordic(SMO->ThetaE, &SMO->SinTheta, &SMO->CosTheta);

    float SMO_SpdE_Temp = 0;
    float SMO_ThetaE_temp = 0;

    switch(SMO->status){
        case 0:
            SMO->ThetaE = MRT_Inf->ThetaE;
            SMO->SpdE = MRT_Inf->Spd * MotorParameter->Np;
            SMO->PLL_PI.ui = SMO->SpdE;
            break;
        case 1:
        case 2:
            SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta);
            SMO_SpdE_Temp = PID_Control_Err(&(SMO->PLL_PI), SMO->de);
            LPF(&(SMO->SpdE), SMO_SpdE_Temp, CtrlCom->CurFs, SMO->Spd_LPF_wc);

            SMO_ThetaE_temp = SMO->ThetaE + SMO_SpdE_Temp * CtrlCom->CurTs;
            if(SMO_ThetaE_temp < 0)
                SMO_ThetaE_temp += 2 * PI;
            SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
            break;
    }

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void HSMO_Mode4(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    SMO->Ex = SMO->Ex + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey + SMO->h2 * SMO->Vx / MotorParameter->Ls);
    SMO->Ey = SMO->Ey + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex + SMO->h2 * SMO->Vy / MotorParameter->Ls);

    if(SMO->Ex > SMO->Switch_EMF * 0.25f)
        SMO->Qx = 1;
    else if(SMO->Ex < -SMO->Switch_EMF * 0.25f)
        SMO->Qx = 0;

    if(SMO->Ey > SMO->Switch_EMF * 0.25f)
        SMO->Qy = 1;
    else if(SMO->Ey < -SMO->Switch_EMF * 0.25f)
        SMO->Qy = 0;

    switch (SMO->Dir_status){
        case 0:
            if(SMO->Qx == 1){
                SMO->Dir_status = 1;
            }

            if(SMO->Qy == 1){
                SMO->Dir_status = 3;
            }
            break;
        case 1:
            if(SMO->Qy == 1){
                SMO->Dir_status = 2;
                SMO->Dir = 1;
            }

            if(SMO->Qx == 0){
                SMO->Dir_status = 0;
            }
            break;
        case 2:
            if(SMO->Qx == 0){
                SMO->Dir_status = 3;
            }

            if(SMO->Qy == 0){
                SMO->Dir_status = 1;
            }
            break;
        case 3:
            if(SMO->Qx == 1){
                SMO->Dir_status = 2;
                SMO->Dir = -1;
            }

            if(SMO->Qy == 0){
                SMO->Dir_status = 0;
            }
            break;
    }

    SMO->EMF_Rms = sqrtf(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey);

    switch(SMO->status){
        case 0:
            if(SMO->EMF_Rms > SMO->Switch_EMF * 0.5f){
                SMO->status = 1;
            }
            else{
                SMO->status = 0;
            }
            break;
        case 1:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else if(SMO->EMF_Rms > SMO->Switch_EMF){
                SMO->status = 2;
            }
            else{
                SMO->status = 1;
            }
            break;
        case 2:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else{
                SMO->status = 2;
            }
            break;
    }

    Cordic(SMO->ThetaE, &SMO->SinTheta, &SMO->CosTheta);

    float SMO_SpdE_Temp = 0;
    float SMO_ThetaE_temp = 0;

    switch(SMO->status){
        case 0:
            SMO->ThetaE = MRT_Inf->ThetaE;
            SMO->SpdE = MRT_Inf->Spd * MotorParameter->Np;
            SMO->NPLL_PI.ui = SMO->SpdE;
            break;
        case 1:
            if(SMO->Dir == 0){
                SMO->ThetaE = MRT_Inf->ThetaE;
                SMO->SpdE = MRT_Inf->Spd * MotorParameter->Np;
                SMO->NPLL_PI.ui = SMO->SpdE;
            }
            else{
                SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->Switch_EMF * SMO->Dir;
                SMO_SpdE_Temp = PID_Control_Err(&(SMO->NPLL_PI), SMO->de);
                LPF(&(SMO->SpdE), SMO_SpdE_Temp, CtrlCom->CurFs, SMO->Spd_LPF_wc);

                SMO_ThetaE_temp = SMO->ThetaE + SMO_SpdE_Temp * CtrlCom->CurTs;
                if(SMO_ThetaE_temp < 0)
                    SMO_ThetaE_temp += 2 * PI;
                SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
            }
            break;
        case 2:
            if(SMO->Dir == 0){
                SMO->ThetaE = MRT_Inf->ThetaE;
                SMO->SpdE = MRT_Inf->Spd * MotorParameter->Np;
                SMO->NPLL_PI.ui = SMO->SpdE;
            }
            else{
                SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->EMF_Rms  * SMO->Dir;
                SMO_SpdE_Temp = PID_Control_Err(&(SMO->NPLL_PI), SMO->de);
                LPF(&(SMO->SpdE), SMO_SpdE_Temp, CtrlCom->CurFs, SMO->Spd_LPF_wc);

                SMO_ThetaE_temp = SMO->ThetaE + SMO_SpdE_Temp * CtrlCom->CurTs;
                if(SMO_ThetaE_temp < 0)
                    SMO_ThetaE_temp += 2 * PI;
                SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
            }
            break;
    }

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void HSMO_Mode5(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * SMO->Ix - SMO->Ex - SMO->h1 * SMO->Vx) / MotorParameter->Ls;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * SMO->Iy - SMO->Ey - SMO->h1 * SMO->Vy) / MotorParameter->Ls;

    SMO->Ex = SMO->Ex + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey + SMO->h2 * SMO->Vx / MotorParameter->Ls);
    SMO->Ey = SMO->Ey + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex + SMO->h2 * SMO->Vy / MotorParameter->Ls);

    if(SMO->Ex > SMO->Switch_EMF * 0.25f)
        SMO->Qx = 1;
    else if((SMO->Ex <= SMO->Switch_EMF * 0.25f) && (SMO->Ex > -SMO->Switch_EMF * 0.25f))
        SMO->Qx = 0;
    else if(SMO->Ex <= -SMO->Switch_EMF * 0.25f)
        SMO->Qx = -1;

    if(SMO->Ey > SMO->Switch_EMF * 0.25f)
        SMO->Qy = 1;
    else if((SMO->Ey <= SMO->Switch_EMF * 0.25f) && (SMO->Ey > -SMO->Switch_EMF * 0.25f))
        SMO->Qy = 0;
    else if(SMO->Ey <= -SMO->Switch_EMF * 0.25f)
        SMO->Qy = -1;

    switch (SMO->Dir_status){
        case 0: //YX: 0  0
            if(SMO->Qy == 1){
                SMO->Dir_status = 1;
            }
            else if(SMO->Qy == -1){
                SMO->Dir_status = 5;
            }
            else if(SMO->Qx == -1){
                SMO->Dir_status = 3;
            }
            else if(SMO->Qx == 1){
                SMO->Dir_status = 7;
            }
            break;

        case 1: //YX: 1  0
            if(SMO->Qx == -1){
                SMO->Dir_status = 2;
                SMO->Dir = 1;
            }
            else if(SMO->Qx == 1){
                SMO->Dir_status = 8;
                SMO->Dir = -1;
            }
            break;

        case 2: //YX: 1 -1
            if(SMO->Qx == 0){
                SMO->Dir_status = 1;
            }
            else if(SMO->Qx == 1){
                SMO->Dir_status = 8;
            }
            else if(SMO->Qy == 0){
                SMO->Dir_status = 3;
            }
            else if(SMO->Qy == -1){
                SMO->Dir_status = 4;
            }
            break;

        case 3: //YX: 0 -1
            if(SMO->Qy == -1){
                SMO->Dir_status = 4;
                SMO->Dir = 1;
            }
            else if(SMO->Qy == 1){
                SMO->Dir_status = 2;
                SMO->Dir = -1;
            }
            break;
        case 4: //YX:-1 -1
            if(SMO->Qx == 0){
                SMO->Dir_status = 5;
            }
            else if(SMO->Qx == 1){
                SMO->Dir_status = 6;
            }
            else if(SMO->Qy == 0){
                SMO->Dir_status = 3;
            }
            else if(SMO->Qy == 1){
                SMO->Dir_status = 2;
            }
            break;
        case 5: //YX:-1  0
            if(SMO->Qx == 1){
                SMO->Dir_status = 6;
                SMO->Dir = 1;
            }
            else if(SMO->Qx == -1){
                SMO->Dir_status = 4;
                SMO->Dir = -1;
            }
            break;
        case 6: //YX:-1  1
            if(SMO->Qx == 0){
                SMO->Dir_status = 5;
            }
            else if(SMO->Qx == -1){
                SMO->Dir_status = 4;
            }
            else if(SMO->Qy == 0){
                SMO->Dir_status = 7;
            }
            else if(SMO->Qy == 1){
                SMO->Dir_status = 8;
            }
            break;
        case 7: //YX: 0  1
            if(SMO->Qy == 1){
                SMO->Dir_status = 8;
                SMO->Dir = 1;
            }
            else if(SMO->Qy == -1){
                SMO->Dir_status = 6;
                SMO->Dir = -1;
            }
            break;
        case 8: //YX: 1  1
            if(SMO->Qx == 0){
                SMO->Dir_status = 1;
            }
            else if(SMO->Qx == -1){
                SMO->Dir_status = 2;
            }
            else if(SMO->Qy == 0){
                SMO->Dir_status = 7;
            }
            else if(SMO->Qy == -1){
                SMO->Dir_status = 6;
            }
            break;
    }

    SMO->EMF_Rms = sqrtf(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey);

    switch(SMO->status){
        case 0:
            if(SMO->EMF_Rms > SMO->Switch_EMF * 0.5f){
                SMO->status = 1;
            }
            else{
                SMO->status = 0;
            }
            break;
        case 1:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else if(SMO->EMF_Rms > SMO->Switch_EMF){
                SMO->status = 2;
            }
            else{
                SMO->status = 1;
            }
            break;
        case 2:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else{
                SMO->status = 2;
            }
            break;
    }

    Cordic(SMO->ThetaE, &SMO->SinTheta, &SMO->CosTheta);

    float SMO_SpdE_Temp = 0;
    float SMO_ThetaE_temp = 0;

    switch(SMO->status){
        case 0:
            SMO->ThetaE = MRT_Inf->ThetaE;
            SMO->SpdE = MRT_Inf->Spd * MotorParameter->Np;
            SMO->NPLL_PI.ui = SMO->SpdE;
            break;
        case 1:
        case 2:
            SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->EMF_Rms  * SMO->Dir;
            SMO_SpdE_Temp = PID_Control_Err(&(SMO->NPLL_PI), SMO->de);
            LPF(&(SMO->SpdE), SMO_SpdE_Temp, CtrlCom->CurFs, SMO->Spd_LPF_wc);

            SMO_ThetaE_temp = SMO->ThetaE + SMO_SpdE_Temp * CtrlCom->CurTs;
            if(SMO_ThetaE_temp < 0)
                SMO_ThetaE_temp += 2 * PI;
            SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
            break;
    }

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void HSMO_Mode6(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    D_PI->Ki *= CtrlCom->CurTs;
    Q_PI->Ki *= CtrlCom->CurTs;
    Spd_PI->Ki *= CtrlCom->SpdTs;

    MRT_Inf->ThetaE = GetThetaE(MRT_Inf->Theta, MotorParameter->Np);
    
    GetSpd(MRT_Inf->Theta, &CtrlCom->Theta_Pre, CtrlCom->Spd_Tick, &MRT_Inf->Spd, CtrlCom->SpdTs, &CtrlCom->Start_Flag);
    Clarke(MRT_Inf->Ia, MRT_Inf->Ic, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;

    SMO->Vx = SMOSwitchFunction1(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction1(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * MRT_Inf->Ix - SMO->SpdE * (MotorParameter->Ld - MotorParameter->Lq) * MRT_Inf->Iy - SMO->Ex_Test - SMO->h1 * SMO->Vx) / MotorParameter->Ld;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * MRT_Inf->Iy + SMO->SpdE * (MotorParameter->Ld - MotorParameter->Lq) * MRT_Inf->Ix - SMO->Ey_Test - SMO->h1 * SMO->Vy) / MotorParameter->Ld;

    SMO->Ex_Test = SMO->Ex_Test + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey_Test + SMO->h2 * SMO->Vx / MotorParameter->Ld);
    SMO->Ey_Test = SMO->Ey_Test + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex_Test + SMO->h2 * SMO->Vy / MotorParameter->Ld);

    LPF(&(SMO->Ex), SMO->Ex_Test, CtrlCom->CurFs, SMO->Spd_LPF_wc * 4.5);
    LPF(&(SMO->Ey), SMO->Ey_Test, CtrlCom->CurFs, SMO->Spd_LPF_wc * 4.5);

    if(SMO->Ex > SMO->Switch_EMF * 0.25f)
        SMO->Qx = 1;
    else if((SMO->Ex <= SMO->Switch_EMF * 0.25f) && (SMO->Ex > -SMO->Switch_EMF * 0.25f))
        SMO->Qx = 0;
    else if(SMO->Ex <= -SMO->Switch_EMF * 0.25f)
        SMO->Qx = -1;

    if(SMO->Ey > SMO->Switch_EMF * 0.25f)
        SMO->Qy = 1;
    else if((SMO->Ey <= SMO->Switch_EMF * 0.25f) && (SMO->Ey > -SMO->Switch_EMF * 0.25f))
        SMO->Qy = 0;
    else if(SMO->Ey <= -SMO->Switch_EMF * 0.25f)
        SMO->Qy = -1;

    switch (SMO->Dir_status){
        case 0: //YX: 0  0
            if(SMO->Qy == 1){
                SMO->Dir_status = 1;
            }
            else if(SMO->Qy == -1){
                SMO->Dir_status = 5;
            }
            else if(SMO->Qx == -1){
                SMO->Dir_status = 3;
            }
            else if(SMO->Qx == 1){
                SMO->Dir_status = 7;
            }
            break;

        case 1: //YX: 1  0
            if(SMO->Qx == -1){
                SMO->Dir_status = 2;
                SMO->Dir = 1;
            }
            else if(SMO->Qx == 1){
                SMO->Dir_status = 8;
                SMO->Dir = -1;
            }
            break;

        case 2: //YX: 1 -1
            if(SMO->Qx == 0){
                SMO->Dir_status = 1;
            }
            else if(SMO->Qx == 1){
                SMO->Dir_status = 8;
            }
            else if(SMO->Qy == 0){
                SMO->Dir_status = 3;
            }
            else if(SMO->Qy == -1){
                SMO->Dir_status = 4;
            }
            break;

        case 3: //YX: 0 -1
            if(SMO->Qy == -1){
                SMO->Dir_status = 4;
                SMO->Dir = 1;
            }
            else if(SMO->Qy == 1){
                SMO->Dir_status = 2;
                SMO->Dir = -1;
            }
            break;
        case 4: //YX:-1 -1
            if(SMO->Qx == 0){
                SMO->Dir_status = 5;
            }
            else if(SMO->Qx == 1){
                SMO->Dir_status = 6;
            }
            else if(SMO->Qy == 0){
                SMO->Dir_status = 3;
            }
            else if(SMO->Qy == 1){
                SMO->Dir_status = 2;
            }
            break;
        case 5: //YX:-1  0
            if(SMO->Qx == 1){
                SMO->Dir_status = 6;
                SMO->Dir = 1;
            }
            else if(SMO->Qx == -1){
                SMO->Dir_status = 4;
                SMO->Dir = -1;
            }
            break;
        case 6: //YX:-1  1
            if(SMO->Qx == 0){
                SMO->Dir_status = 5;
            }
            else if(SMO->Qx == -1){
                SMO->Dir_status = 4;
            }
            else if(SMO->Qy == 0){
                SMO->Dir_status = 7;
            }
            else if(SMO->Qy == 1){
                SMO->Dir_status = 8;
            }
            break;
        case 7: //YX: 0  1
            if(SMO->Qy == 1){
                SMO->Dir_status = 8;
                SMO->Dir = 1;
            }
            else if(SMO->Qy == -1){
                SMO->Dir_status = 6;
                SMO->Dir = -1;
            }
            break;
        case 8: //YX: 1  1
            if(SMO->Qx == 0){
                SMO->Dir_status = 1;
            }
            else if(SMO->Qx == -1){
                SMO->Dir_status = 2;
            }
            else if(SMO->Qy == 0){
                SMO->Dir_status = 7;
            }
            else if(SMO->Qy == -1){
                SMO->Dir_status = 6;
            }
            break;
    }

    SMO->EMF_Rms = sqrtf(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey);

    switch(SMO->status){
        case 0:
            if(SMO->EMF_Rms > SMO->Switch_EMF * 0.5f){
                SMO->status = 1;
            }
            else{
                SMO->status = 0;
            }
            break;
        case 1:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else if(SMO->EMF_Rms > SMO->Switch_EMF){
                SMO->status = 2;
            }
            else{
                SMO->status = 1;
            }
            break;
        case 2:
            if(SMO->EMF_Rms < SMO->Switch_EMF * 0.25f){
                SMO->status = 0;
            }
            else{
                SMO->status = 2;
            }
            break;
    }

    Cordic(SMO->ThetaE, &SMO->SinTheta, &SMO->CosTheta);

    float SMO_SpdE_Temp = 0;
    float SMO_ThetaE_temp = 0;

    switch(SMO->status){
        case 0:
            SMO->ThetaE = MRT_Inf->ThetaE;
            SMO->SpdE = MRT_Inf->Spd * MotorParameter->Np;
            SMO->NPLL_PI.ui = SMO->SpdE;
            break;
        case 1:
        case 2:
            SMO->de = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->EMF_Rms  * SMO->Dir;
            SMO_SpdE_Temp = PID_Control_Err(&(SMO->NPLL_PI), SMO->de);
            LPF(&(SMO->SpdE), SMO_SpdE_Temp, CtrlCom->CurFs, SMO->Spd_LPF_wc);

            SMO_ThetaE_temp = SMO->ThetaE + SMO_SpdE_Temp * CtrlCom->CurTs;
            if(SMO_ThetaE_temp < 0)
                SMO_ThetaE_temp += 2 * PI;
            SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
            break;
    }

    Park(MRT_Inf->Ix, MRT_Inf->Iy, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Id, &MRT_Inf->Iq);

    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        CtrlCom->Iq = PID_Control(Spd_PI, CtrlCom->Spd, MRT_Inf->Spd);
    }

    MRT_Inf->Ud = PID_Control(D_PI, CtrlCom->Id, MRT_Inf->Id);
    MRT_Inf->Uq = PID_Control(Q_PI, CtrlCom->Iq, MRT_Inf->Iq);

    InvPark(MRT_Inf->Ud, MRT_Inf->Uq, MRT_Inf->SinTheta, MRT_Inf->CosTheta, &MRT_Inf->Ux, &MRT_Inf->Uy);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    MRT_Inf->Sector = GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Udc, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void FOC(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO){
    Spd_Timer(&(CtrlCom->Spd_Tick));
    
    if(CtrlCom->Mode == 0){
        SpeedLoopOri_Mode0(CtrlCom, MotorParameter, MRT_Inf);
    }
    else if(CtrlCom->Mode == 1){
        CurLoop_Mode1(D_PI, Q_PI, CtrlCom, MotorParameter, MRT_Inf);
    }
    else if(CtrlCom->Mode == 2){
        SpeedLoopOri_Mode2(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf);
    }
    else if(CtrlCom->Mode == 3){
        SMO_Mode1(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 4){
        HSMO_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 5){
        SMO_Mode0(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 6){
        SMO_Mode2(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 7){
        HSMO_Mode2(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 8){
        HSMO_Mode3(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 9){
        HSMO_Mode4(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 10){
        HSMO_Mode5(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
    else if(CtrlCom->Mode == 11){
        HSMO_Mode6(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO);
    }
}
