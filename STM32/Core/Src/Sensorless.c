#include "Sensorless.h"
#include "FOCSub.h"
#include "DataProcessing.h"

void Ih_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    float HFI_Ih = 0;
    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);
    
    HFI->Ixh = (HFI->Ix - HFI->Ix_temp) / 2 * HFI->Dir;
    HFI->Iyh = (HFI->Iy - HFI->Iy_temp) / 2 * HFI->Dir;
    arm_sqrt_f32(HFI->Ixh * HFI->Ixh + HFI->Iyh * HFI->Iyh, &HFI_Ih);
    LPF(&HFI->Ih, HFI_Ih, CtrlCom->CurFs, HFI->Ih_LPF_wc);
    HFI->Ih_Err = HFI_Ih - HFI->Ih;
    
    HFI->Dir = (HFI->Dir == 1)?(-1):(1);
    HFI->Vdh = HFI->Dir * HFI->Vh;
    
    HFI->Ud = HFI->Vdh;
    HFI->Uq = 0;
    
    arm_inv_park_f32(HFI->Ud, HFI->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, 0, 1);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
    
    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
}

void ThetaE_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    float HFI_SpdE = 0;
    float HFI_SpdE_Rec = 0;
    float HFI_Ih = 0;
    float HFI_ThetaE_temp = 0;
    float HFI_ThetaE_Rec_temp = 0;
    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);
    
    HFI->Ixh = (HFI->Ix - HFI->Ix_temp) / 2 * HFI->Dir;
    HFI->Iyh = (HFI->Iy - HFI->Iy_temp) / 2 * HFI->Dir;

    arm_sqrt_f32(HFI->Ixh * HFI->Ixh + HFI->Iyh * HFI->Iyh, &HFI_Ih);
    LPF(&HFI->Ih, HFI_Ih, CtrlCom->CurFs, HFI->Ih_LPF_wc);
    Cordic(HFI->ThetaE, &HFI->SinTheta, &HFI->CosTheta);

    HFI->SpdE_PI.Error = (HFI->Ixh * HFI->SinTheta - HFI->Iyh * HFI->CosTheta) / HFI->Ih;
    LPF(&(HFI->ThetaE_Err), HFI->SpdE_PI.Error, CtrlCom->CurFs, HFI->ThetaE_Err_LPF_wc);
    
    HFI_SpdE = PI_Control(&HFI->SpdE_PI);
    HFI_ThetaE_temp = HFI->ThetaE + HFI_SpdE * CtrlCom->CurTs;
    if(HFI_ThetaE_temp < 0)
        HFI_ThetaE_temp += 2 * PI;
    HFI->ThetaE = fmodf(HFI_ThetaE_temp, 2 * PI);
    
    Cordic(fmodf(2.0f * HFI_ThetaE_temp + HFI->Rec.c1, 2*PI), &HFI->SinThetaE_Rec, &HFI->CosThetaE_Rec);
    HFI_ThetaE_Rec_temp = HFI_ThetaE_temp + HFI->Rec.a1*HFI->SinThetaE_Rec + HFI->Rec.d1 + HFI->Rec.d2 * MRT_Inf->Iq_Ave / 2.7f;
    if(HFI_ThetaE_Rec_temp < 0)
        HFI_ThetaE_Rec_temp += 2 * PI;
    HFI->ThetaE_Rec = fmodf(HFI_ThetaE_Rec_temp, 2 * PI);
    
    HFI_SpdE_Rec = HFI->ThetaE_Rec - HFI->ThetaE_Rec_temp;
    if(HFI_SpdE_Rec > PI)
        HFI_SpdE_Rec -= 2 * PI;
    else if(HFI_SpdE_Rec < -PI)
        HFI_SpdE_Rec += 2 * PI;
    HFI_SpdE_Rec = HFI_SpdE_Rec / CtrlCom->CurTs;
    LPF(&(HFI->SpdE_Rec), HFI_SpdE_Rec, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
    HFI->Dir = (HFI->Dir == 1)?(-1):(1);
    HFI->Vdh = HFI->Dir * HFI->Vh;
    
    HFI->Ud = HFI->Vdh;
    HFI->Uq = 0;
    
    arm_inv_park_f32(HFI->Ud, HFI->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, HFI->SinTheta, HFI->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
    
    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
    HFI->ThetaE_Rec_temp = HFI->ThetaE_Rec;
}

void Pole_Init(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    Cordic(HFI->ThetaE, &HFI->SinTheta, &HFI->CosTheta);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, HFI->SinTheta, HFI->CosTheta);
    
    if(HFI->Pulse_cnt < 1){
        HFI->Pulse_cnt++;
        HFI->Ud = HFI->Ud;
        HFI->Uq = 0;
        
        if(HFI->Ipulse_Max < MRT_Inf->Id){
            HFI->Ipulse_Max = MRT_Inf->Id;
        }
        
        if(HFI->Ipulse_Min > MRT_Inf->Id){
            HFI->Ipulse_Min = MRT_Inf->Id;
        }   
    }
    else if(HFI->Pulse_cnt < 124){
        HFI->Pulse_cnt++;
        HFI->Ud = 0;
        HFI->Uq = 0;
        
        if(HFI->Ipulse_Max < MRT_Inf->Id){
            HFI->Ipulse_Max = MRT_Inf->Id;
        }
        
        if(HFI->Ipulse_Min > MRT_Inf->Id){
            HFI->Ipulse_Min = MRT_Inf->Id;
        }
    }
    else{
        HFI->Pulse_cnt = 0;
        HFI->Dir = (HFI->Dir == 1)?(-1):(1);
        HFI->Ud = HFI->VPulse * HFI->Dir;
        HFI->Uq = 0;
        if(HFI->Dir == 1){
            HFI->Ipulse_Max = 0;
        }
        else{
            HFI->Ipulse_Min = 0;
        }
    }
    
    arm_inv_park_f32(HFI->Ud, HFI->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, HFI->SinTheta, HFI->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

float SMOSwitchFunction_Sat(float E, float Error){
    float SF_Out;

    SF_Out = Error / E;
    
    if(SF_Out > 1)
        SF_Out = 1;
    else if(SF_Out < -1)
        SF_Out = -1;

    return SF_Out;
}

void SMO_Dir_Get(SlidingModeObserver_str* SMO){
    if(SMO->Ex_Test > SMO->Switch_EMF * 0.25f){
        SMO->Qx = 1;
    }
    else if((SMO->Ex_Test <= SMO->Switch_EMF * 0.25f) && (SMO->Ex_Test > -SMO->Switch_EMF * 0.25f)){
        SMO->Qx = 0;
    }   
    else if(SMO->Ex_Test <= -SMO->Switch_EMF * 0.25f){
        SMO->Qx = -1;
    }

    if(SMO->Ey_Test > SMO->Switch_EMF * 0.25f){
        SMO->Qy = 1;
    }
    else if((SMO->Ey_Test <= SMO->Switch_EMF * 0.25f) && (SMO->Ey_Test > -SMO->Switch_EMF * 0.25f)){
        SMO->Qy = 0;
    }
    else if(SMO->Ey_Test <= -SMO->Switch_EMF * 0.25f){
        SMO->Qy = -1;
    }

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
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qx == 1){
            SMO->Dir_status = 8;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        break;

    case 2: //YX: 1 -1
        if(SMO->Qx == 0){
            SMO->Dir_status = 1;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        else if(SMO->Qx == 1){
            SMO->Dir_status = 8;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        else if(SMO->Qy == 0){
            SMO->Dir_status = 3;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qy == -1){
            SMO->Dir_status = 4;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        break;

    case 3: //YX: 0 -1
        if(SMO->Qy == -1){
            SMO->Dir_status = 4;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qy == 1){
            SMO->Dir_status = 2;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        break;
    case 4: //YX:-1 -1
        if(SMO->Qx == 0){
            SMO->Dir_status = 5;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qx == 1){
            SMO->Dir_status = 6;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qy == 0){
            SMO->Dir_status = 3;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        else if(SMO->Qy == 1){
            SMO->Dir_status = 2;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        break;
    case 5: //YX:-1  0
        if(SMO->Qx == 1){
            SMO->Dir_status = 6;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qx == -1){
            SMO->Dir_status = 4;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        break;
    case 6: //YX:-1  1
        if(SMO->Qx == 0){
            SMO->Dir_status = 5;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        else if(SMO->Qx == -1){
            SMO->Dir_status = 4;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        else if(SMO->Qy == 0){
            SMO->Dir_status = 7;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qy == 1){
            SMO->Dir_status = 8;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        break;
    case 7: //YX: 0  1
        if(SMO->Qy == 1){
            SMO->Dir_status = 8;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qy == -1){
            SMO->Dir_status = 6;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        break;
    case 8: //YX: 1  1
        if(SMO->Qx == 0){
            SMO->Dir_status = 1;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qx == -1){
            SMO->Dir_status = 2;
            SMO->DirP++;
            SMO->DirN = 0;
        }
        else if(SMO->Qy == 0){
            SMO->Dir_status = 7;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        else if(SMO->Qy == -1){
            SMO->Dir_status = 6;
            SMO->DirP = 0;
            SMO->DirN++;
        }
        break;
    }

    if(SMO->DirP >= 2){
        SMO->Dir = 1;
    }
    else if(SMO->DirN >= 2){
        SMO->Dir = -1;
    }
}

void HFI_Work(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    float HFI_SpdE = 0;
    float HFI_SpdE_Rec = 0;
    float HFI_Ih = 0;
    float HFI_ThetaE_temp = 0;
    float SMO_EMF_Rms;
    float SMO_EMF_Rms_Test;

    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;
    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);
    
    MRT_Inf->Ix = (HFI->Ix + HFI->Ix_temp) / 2;
    MRT_Inf->Iy = (HFI->Iy + HFI->Iy_temp) / 2;
    HFI->Ixh = (HFI->Ix - HFI->Ix_temp) / 2 * HFI->Dir;
    HFI->Iyh = (HFI->Iy - HFI->Iy_temp) / 2 * HFI->Dir;
    arm_sqrt_f32(HFI->Ixh * HFI->Ixh + HFI->Iyh * HFI->Iyh, &HFI_Ih);
    LPF(&HFI->Ih, HFI_Ih, CtrlCom->CurFs, HFI->Ih_LPF_wc);
    
    Cordic(HFI->ThetaE, &HFI->SinTheta, &HFI->CosTheta);
    HFI->SpdE_PI.Error = (HFI->Ixh * HFI->SinTheta - HFI->Iyh * HFI->CosTheta) / HFI->Ih;

    HFI_SpdE = PI_Control(&HFI->SpdE_PI);  
    HFI_ThetaE_temp = HFI->ThetaE + HFI_SpdE * CtrlCom->CurTs;
    if(HFI_ThetaE_temp < 0)
        HFI_ThetaE_temp += 2 * PI;
    HFI->ThetaE = fmodf(HFI_ThetaE_temp, 2 * PI);

    HFI->ThetaE_Rec = HFI_angle_Rec3(MRT_Inf, HFI);
    
    Cordic(HFI->ThetaE_Rec, &HFI->SinTheta_Rec, &HFI->CosTheta_Rec);
    
    HFI_SpdE_Rec = HFI->ThetaE_Rec - HFI->ThetaE_Rec_temp;
    if(HFI_SpdE_Rec > PI)
        HFI_SpdE_Rec -= 2*PI;
    else if(HFI_SpdE_Rec < -PI)
        HFI_SpdE_Rec += 2*PI;
    HFI_SpdE_Rec = HFI_SpdE_Rec / CtrlCom->CurTs;
    LPF(&(HFI->SpdE_Rec), HFI_SpdE_Rec, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
    SMO->Vx = SMOSwitchFunction_Sat(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction_Sat(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * MRT_Inf->Ix - SMO->SpdE * (MotorParameter->Ld - MotorParameter->Lq) * MRT_Inf->Iy - SMO->Ex_Test - SMO->h1 * SMO->Vx) / MotorParameter->Ld;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * MRT_Inf->Iy + SMO->SpdE * (MotorParameter->Ld - MotorParameter->Lq) * MRT_Inf->Ix - SMO->Ey_Test - SMO->h1 * SMO->Vy) / MotorParameter->Ld;

    SMO->Ex_Test = SMO->Ex_Test + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey_Test + SMO->h2 * SMO->Vx / MotorParameter->Ld);
    SMO->Ey_Test = SMO->Ey_Test + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex_Test + SMO->h2 * SMO->Vy / MotorParameter->Ld);

    arm_sqrt_f32(SMO->Ex_Test * SMO->Ex_Test + SMO->Ey_Test * SMO->Ey_Test, &SMO_EMF_Rms_Test);
    LPF(&(SMO->EMF_Rms_Test), SMO_EMF_Rms_Test, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    
    LPF(&(SMO->Ex), SMO->Ex_Test, CtrlCom->CurFs, SMO->Exy_LPF_wc);
    LPF(&(SMO->Ey), SMO->Ey_Test, CtrlCom->CurFs, SMO->Exy_LPF_wc);

    arm_sqrt_f32(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey, &SMO_EMF_Rms);
    LPF(&(SMO->EMF_Rms), SMO_EMF_Rms, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    
    SMO_Dir_Get(SMO);
    
    SMO->SpdE = HFI->SpdE_Rec;
    SMO->Spd = SMO->SpdE / MotorParameter->Np;
    SMO->ThetaE = HFI->ThetaE_Rec;
    
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, HFI->SinTheta_Rec, HFI->CosTheta_Rec);
    LPF(&(MRT_Inf->Iq_Ave), MRT_Inf->Iq, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    
    HFI->Dir = (HFI->Dir == 1)?(-1):(1);
    HFI->Vdh = HFI->Dir * HFI->Vh;
    
    HFI->Spd_Rec = HFI->SpdE_Rec / MotorParameter->Np;
    
    CtrlCom->Id = 0;
    Spd_PI->Error = CtrlCom->Spd - HFI->Spd_Rec;
    CtrlCom->Iq = PI_Control(Spd_PI);
    
    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, HFI->SinTheta_Rec, HFI->CosTheta_Rec);
    arm_inv_park_f32(HFI->Vdh, 0, &HFI->Ux, &HFI->Uy, HFI->SinTheta, HFI->CosTheta);
    InvClarke(MRT_Inf->Ux + HFI->Ux, MRT_Inf->Uy + HFI->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);

    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
    
    HFI->ThetaE_Rec_temp = HFI->ThetaE_Rec;
}

void HFI_SMO_Mix_Work(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    float HFI_SpdE = 0;
    float HFI_SpdE_Rec = 0;
    float HFI_Ih = 0;
    float HFI_ThetaE_temp = 0;
    float SMO_EMF_Rms;
    float SMO_EMF_Rms_Test;

    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;
    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);
    
    MRT_Inf->Ix = (HFI->Ix + HFI->Ix_temp) / 2;
    MRT_Inf->Iy = (HFI->Iy + HFI->Iy_temp) / 2;
    HFI->Ixh = (HFI->Ix - HFI->Ix_temp) / 2 * HFI->Dir;
    HFI->Iyh = (HFI->Iy - HFI->Iy_temp) / 2 * HFI->Dir;
    arm_sqrt_f32(HFI->Ixh * HFI->Ixh + HFI->Iyh * HFI->Iyh, &HFI_Ih);
    LPF(&HFI->Ih, HFI_Ih, CtrlCom->CurFs, HFI->Ih_LPF_wc);

    Cordic(HFI->ThetaE, &HFI->SinTheta, &HFI->CosTheta);

    HFI->SpdE_PI.Error = (HFI->Ixh * HFI->SinTheta - HFI->Iyh * HFI->CosTheta) / HFI->Ih;
    HFI_SpdE = PI_Control(&HFI->SpdE_PI);  
    HFI_ThetaE_temp = HFI->ThetaE + HFI_SpdE * CtrlCom->CurTs;
    if(HFI_ThetaE_temp < 0)
        HFI_ThetaE_temp += 2 * PI;
    HFI->ThetaE = fmodf(HFI_ThetaE_temp, 2 * PI);
    
    HFI->ThetaE_Rec = HFI_angle_Rec3(MRT_Inf, HFI);
    
    Cordic(HFI->ThetaE_Rec, &HFI->SinTheta_Rec, &HFI->CosTheta_Rec);
    
    HFI_SpdE_Rec = HFI->ThetaE_Rec - HFI->ThetaE_Rec_temp;
    if(HFI_SpdE_Rec > PI)
        HFI_SpdE_Rec -= 2*PI;
    else if(HFI_SpdE_Rec < -PI)
        HFI_SpdE_Rec += 2*PI;
    HFI_SpdE_Rec = HFI_SpdE_Rec / CtrlCom->CurTs;
    LPF(&(HFI->SpdE_Rec), HFI_SpdE_Rec, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
    SMO->Vx = SMOSwitchFunction_Sat(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction_Sat(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * MRT_Inf->Ix - SMO->SpdE * (MotorParameter->Ld - MotorParameter->Lq) * MRT_Inf->Iy - SMO->Ex_Test - SMO->h1 * SMO->Vx) / MotorParameter->Ld;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * MRT_Inf->Iy + SMO->SpdE * (MotorParameter->Ld - MotorParameter->Lq) * MRT_Inf->Ix - SMO->Ey_Test - SMO->h1 * SMO->Vy) / MotorParameter->Ld;

    SMO->Ex_Test = SMO->Ex_Test + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey_Test + SMO->h2 * SMO->Vx / MotorParameter->Ld);
    SMO->Ey_Test = SMO->Ey_Test + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex_Test + SMO->h2 * SMO->Vy / MotorParameter->Ld);

    arm_sqrt_f32(SMO->Ex_Test * SMO->Ex_Test + SMO->Ey_Test * SMO->Ey_Test, &SMO_EMF_Rms_Test);
    LPF(&(SMO->EMF_Rms_Test), SMO_EMF_Rms_Test, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    
    LPF(&(SMO->Ex), SMO->Ex_Test, CtrlCom->CurFs, SMO->Exy_LPF_wc);
    LPF(&(SMO->Ey), SMO->Ey_Test, CtrlCom->CurFs, SMO->Exy_LPF_wc);

    arm_sqrt_f32(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey, &SMO_EMF_Rms);
    LPF(&(SMO->EMF_Rms), SMO_EMF_Rms, CtrlCom->CurFs, SMO->Spd_LPF_wc);

    SMO_Dir_Get(SMO);

    Cordic(SMO->ThetaE, &(SMO->SinTheta), &(SMO->CosTheta));

    SMO->SpdE_PI.Error = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->EMF_Rms * SMO->Dir;
    float SpdE = PI_Control(&(SMO->SpdE_PI));
    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    SMO->Spd = SMO->SpdE / MotorParameter->Np;

    float SMO_ThetaE_temp = SMO->ThetaE + SpdE * CtrlCom->CurTs;
    if(SMO_ThetaE_temp < 0)
        SMO_ThetaE_temp += 2 * PI;
    SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
    
    if(HFI->Run_Status_temp == HFI_WORK){
        arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, HFI->SinTheta_Rec, HFI->CosTheta_Rec);
        LPF(&(MRT_Inf->Iq_Ave), MRT_Inf->Iq, CtrlCom->CurFs, SMO->Spd_LPF_wc);
        
        HFI->Spd_Rec = HFI->SpdE_Rec / MotorParameter->Np;
    
        CtrlCom->Id = 0;
        Spd_PI->Error = CtrlCom->Spd - HFI->Spd_Rec;
        CtrlCom->Iq = PI_Control(Spd_PI);
        
        D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
        MRT_Inf->Ud = PI_Control(D_PI);
        Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
        MRT_Inf->Uq = PI_Control(Q_PI);
        
        arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, HFI->SinTheta_Rec, HFI->CosTheta_Rec);
    }
    else{
        arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, SMO->SinTheta, SMO->CosTheta);
        LPF(&(MRT_Inf->Iq_Ave), MRT_Inf->Iq, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    
        CtrlCom->Id = 0;
        Spd_PI->Error = CtrlCom->Spd - SMO->Spd;
        CtrlCom->Iq = PI_Control(Spd_PI);
        
        D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
        MRT_Inf->Ud = PI_Control(D_PI);
        Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
        MRT_Inf->Uq = PI_Control(Q_PI);
        
        arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, SMO->SinTheta, SMO->CosTheta);
    }
    
    HFI->Dir = (HFI->Dir == 1)?(-1):(1);
    HFI->Vdh = HFI->Dir * HFI->Vh;
    arm_inv_park_f32(HFI->Vdh, 0, &HFI->Ux, &HFI->Uy, HFI->SinTheta, HFI->CosTheta);
    InvClarke(MRT_Inf->Ux + HFI->Ux, MRT_Inf->Uy + HFI->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);

    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
    
    HFI->ThetaE_Rec_temp = HFI->ThetaE_Rec;
}

void SMO_Work(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    float SMO_EMF_Rms;
    float SMO_EMF_Rms_Test;

    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    MRT_Inf->EMF_Peak = MRT_Inf->Spd * MotorParameter->Np * MotorParameter->Flux;
    MRT_Inf->EMF_Rms = MRT_Inf->EMF_Peak / 1.732f * 1.414f;
    MRT_Inf->Ex = -MRT_Inf->EMF_Peak * MRT_Inf->SinTheta;
    MRT_Inf->Ey =  MRT_Inf->EMF_Peak * MRT_Inf->CosTheta;
    
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);
    
    MRT_Inf->Ix = (HFI->Ix + HFI->Ix_temp) / 2;
    MRT_Inf->Iy = (HFI->Iy + HFI->Iy_temp) / 2;
    
    SMO->Vx = SMOSwitchFunction_Sat(SMO->E1, SMO->Ix - MRT_Inf->Ix);
    SMO->Vy = SMOSwitchFunction_Sat(SMO->E1, SMO->Iy - MRT_Inf->Iy);

    SMO->Ix = SMO->Ix + CtrlCom->CurTs * (MRT_Inf->Ux - MotorParameter->Rs * MRT_Inf->Ix - SMO->SpdE * (MotorParameter->Ld - MotorParameter->Lq) * MRT_Inf->Iy - SMO->Ex_Test - SMO->h1 * SMO->Vx) / MotorParameter->Ld;
    SMO->Iy = SMO->Iy + CtrlCom->CurTs * (MRT_Inf->Uy - MotorParameter->Rs * MRT_Inf->Iy + SMO->SpdE * (MotorParameter->Ld - MotorParameter->Lq) * MRT_Inf->Ix - SMO->Ey_Test - SMO->h1 * SMO->Vy) / MotorParameter->Ld;

    SMO->Ex_Test = SMO->Ex_Test + CtrlCom->CurTs * (-SMO->SpdE * SMO->Ey_Test + SMO->h2 * SMO->Vx / MotorParameter->Ld);
    SMO->Ey_Test = SMO->Ey_Test + CtrlCom->CurTs * ( SMO->SpdE * SMO->Ex_Test + SMO->h2 * SMO->Vy / MotorParameter->Ld);

    arm_sqrt_f32(SMO->Ex_Test * SMO->Ex_Test + SMO->Ey_Test * SMO->Ey_Test, &SMO_EMF_Rms_Test);
    LPF(&(SMO->EMF_Rms_Test), SMO_EMF_Rms_Test, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    
    LPF(&(SMO->Ex), SMO->Ex_Test, CtrlCom->CurFs, SMO->Exy_LPF_wc);
    LPF(&(SMO->Ey), SMO->Ey_Test, CtrlCom->CurFs, SMO->Exy_LPF_wc);

    arm_sqrt_f32(SMO->Ex * SMO->Ex + SMO->Ey * SMO->Ey, &SMO_EMF_Rms);
    LPF(&(SMO->EMF_Rms), SMO_EMF_Rms, CtrlCom->CurFs, SMO->Spd_LPF_wc);

    SMO_Dir_Get(SMO);

    // SMO->SpdE_PI.Error = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->EMF_Rms * SMO->EMF_Dir;
    SMO->SpdE_PI.Error = (-SMO->Ex * SMO->CosTheta - SMO->Ey * SMO->SinTheta) / SMO->EMF_Rms * SMO->Dir;
    float SpdE = PI_Control(&(SMO->SpdE_PI));
    LPF(&(SMO->SpdE), SpdE, CtrlCom->CurFs, SMO->Spd_LPF_wc);

    float SMO_ThetaE_temp = SMO->ThetaE + SpdE * CtrlCom->CurTs;
    if(SMO_ThetaE_temp < 0)
        SMO_ThetaE_temp += 2 * PI;
    SMO->ThetaE = fmodf(SMO_ThetaE_temp, 2 * PI);
    Cordic(SMO->ThetaE, &(SMO->SinTheta), &(SMO->CosTheta));
    
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, SMO->SinTheta, SMO->CosTheta);
    LPF(&(MRT_Inf->Iq_Ave), MRT_Inf->Iq, CtrlCom->CurFs, SMO->Spd_LPF_wc);
    
    SMO->Spd = SMO->SpdE / MotorParameter->Np;
    
    CtrlCom->Id = 0;
    Spd_PI->Error = CtrlCom->Spd - SMO->Spd;
    CtrlCom->Iq = PI_Control(Spd_PI);
    
    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, SMO->SinTheta, SMO->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
    
    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
}

const float Iq_const[18] = {-3.0f,      -2.5f,      -2.0f,       -1.5f,        -1.0f,      -0.75f,     -0.5f,       -0.3f,       -0.1f,        0.1f,         0.3f,       0.5f,        0.75f,      1.0f,        1.5f,       2.0f,        2.5f,        3.0f};
const float a0[18]       = {0.3521f,     0.3016f,    0.2508f,     0.2003f,      0.158f,     0.1419f,    0.1296f,     0.1223f,     0.1201f,     0.1528f,      0.1122f,    0.06762f,    0.02492f,  -0.007448f,  -0.07141f,  -0.1357f,    -0.1885f,    -0.2465};
const float a1[18]       = {0.04894f,    0.05341f,   0.06086f,    0.05509f,     0.06353f,   0.07007f,   0.06966f,    0.06485f,    0.01171f,    0.009876f,   -0.01551f,   0.003409f,   0.0171f,    0.02643f,    0.03685f,   0.03451f,    0.04128f,    0.05352f};
const float b1[18]       = {0.05136f,    0.05672f,   0.06322f,    0.0806f,      0.09411f,   0.1087f,    0.1258f,     0.1287f,     0.1298f,     0.1373f,      0.1627f,    0.1596f,     0.1422f,    0.133f,      0.1194f,    0.1102f,     0.1032f,     0.09148f};
const float a2[18]       = {-0.004018f, -0.01067f,  -0.01479f,   -0.02489f,    -0.0351f,   -0.04365f,  -0.0429f,    -0.02458f,    0.01195f,    0.01008f,     0.00549f,   -0.01039f,  -0.01845f,  -0.01431,    -0.01055f,  -0.001743f,   0.003623f,   0.003455f};
const float b2[18]       = {-0.001963f, -0.003114f, -0.0004894f, -0.001868f,   -0.005813f, -0.002974f, -0.009354f,  -0.007603f,   0.01182f,    0.01656f,     0.00213f,   -0.03244f,  -0.03776f,  -0.03508f,   -0.02737f,  -0.01678f,   -0.006321f,   0.004589f};
const float a3[18]       = {-0.002958f, -0.002108f, -0.01039f,    0.0002776f,  -0.001336f, -0.002607f,  0.002784f,   0.001991f,   0.004938f,   0.01087f,    -0.02184f,   -0.004353f,  0.0229f,    0.03142f,    0.02953f,   0.01143f,   -0.001749f,  -0.01332f};
const float b3[18]       = {-0.01018f,  -0.01353f,  -0.01825f,   -0.03017f,    -0.04072f,  -0.03746f,  -0.01768f,    0.0158f,     0.0009665f,  0.004006f,   -0.002671f,  0.00119f,    0.008234f,  0.01408f,    0.02674f,   0.03342f,    0.03275f,    0.02932f};
const float a4[18]       = {0.004969f,   0.003876f,  0.007103f,   0.02219f,     0.02234f,   0.0111f,   -0.00972f,   -0.01065f,    0.0008357f,  -0.0008478f, -0.008059f,  0.01825f,    0.00799f,  -0.009108f,  -0.02752f,  -0.03064f,   -0.02321f,   -0.01771f};
const float b4[18]       = {-0.01101f,  -0.01425f,  -0.01978f,   -0.01372f,    -0.01205f,  -0.008642f,  0.0008059f,  0.004903f,   0.008067f,   -0.0007237f,  0.003154f,  -0.01287f,  -0.001387f,  0.005976f,   0.008205f, -0.008611f,  -0.01581f,   -0.01829f};
const float a5[18]       = {0.007688f,   0.01202f,   0.01664f,    0.01745f,     0.01281f,   0.005244f, -0.003991f,  -0.0005067f, -0.0002398f,  -0.002214f,   0.004339f,  0.0001808f, -0.006058f, -0.007096f,  -0.002487f,  0.01181f,    0.0115f,     0.007547f};
const float b5[18]       = {-0.005368f, -0.004102f, -0.002149f,   0.0107f,      0.008334f, -0.003227f, -0.01151f,    0.009781f,   0.0027f,     0.004078f,   -0.007727f,  0.006365f,   0.01364f,   0.004967f,  -0.0117f,   -0.003048f,   0.0006093f,  0.00157f};
const float a6[18]       = {0.01271f,    0.01009f,   0.009309f,   7.498e-05f,   0.001759f,  0.007618f,  0.0003152f, -0.006087f,   0.0005669f,  0.0001429f,  -0.003026f,  0.002084f,  -0.00537f,   0.001294f,   0.00718f,  -0.002524f,  -0.006979f,  -0.01007f};
const float b6[18]       = {0.002358f,   0.006528f,  0.007553f,   0.01249f,     0.007473f, -0.002482f, -0.008248f,   0.003907f,   0.0002952f,  0.001794f,   -0.005974f,  0.008426f,  -0.005278f, -0.00935f,   -0.002782f,  0.00413f,   -0.001432f,  -0.006454f};
const float a7[18]       = {0.003603,    0.0009412f,-0.001214f,  -0.004459f,   -0.003625f,  0.005652f,  0.003385f,  -0.001343f,   0.00101f,    0.0007897f,   0.002768f,  -0.002637f, -0.0004787f, 0.003861f,  -0.001287f, -0.005119f,   0.002955f,   0.007417f};
const float b7[18]       = {0.00494f,    0.007093f,  0.005942f,   0.001371f,    0.007183f,  0.001337,  -0.006755f,   0.00559f,   -0.000233f,   0.0001835f,  -0.00273f,   0.003204f,  -0.007645f,  0.0006911f,  0.006533f, -0.002822f,  -0.006336f,  -0.004978f};
const float a8[18]       = {-0.001277f, -0.00376f,  -0.002361f,  -8.597e-05f   -0.005266f,  0.002304,   0.006184f,  -0.0002975f, -2.763e-05f,  -0.000378f,   0.004843f,  -0.004518f,  0.00523f,   0.0003428f, -0.003391f,  0.005773f,   0.006319f,   0.003322f};
const float b8[18]       = {0.003405f,   0.002394f, -0.0006058f, -0.002774f,   -0.0004537f, 0.001495,  -0.001048f,   0.003797f,   0.0007483f,  -0.0004975f,  0.0003899f, -0.001819f,  0.001023f,  0.004041f,  -0.00602f,  -0.002127f,   0.001557f,   0.002097f};
const float w [18]       = {1.987f,      1.979f,     1.967f,      2.0f,         1.982f,     1.961f,     1.949f,      1.917f,      2.025f,       2.009f,      2.072f,     2.049f,      2.042f,     2.037f,      2.014f,     2.02f,       2.011f,      1.983f};

float DIq_const[17];
float Da0[17];
float Da1[17];
float Db1[17];
float Da2[17];
float Db2[17];
float Da3[17];
float Db3[17];
float Da4[17];
float Db4[17];
float Da5[17];
float Db5[17];
float Da6[17];
float Db6[17];
float Da7[17];
float Db7[17];
float Da8[17];
float Db8[17];
float Dw [17];

void HFI_angle_Rec3_Init(void){
    for(int8_t i = 0; i < 17; i++){
        DIq_const[i] = Iq_const[i+1] - Iq_const[i];
        Da0[i] = (a0[i+1] - a0[i])/DIq_const[i];
        Da1[i] = (a1[i+1] - a1[i])/DIq_const[i];
        Db1[i] = (b1[i+1] - b1[i])/DIq_const[i];
        Da2[i] = (a2[i+1] - a2[i])/DIq_const[i];
        Db2[i] = (b2[i+1] - b2[i])/DIq_const[i];
        Da3[i] = (a3[i+1] - a3[i])/DIq_const[i];
        Db3[i] = (b3[i+1] - b3[i])/DIq_const[i];
        Da4[i] = (a4[i+1] - a4[i])/DIq_const[i];
        Db4[i] = (b4[i+1] - b4[i])/DIq_const[i];
        Da5[i] = (a5[i+1] - a5[i])/DIq_const[i];
        Db5[i] = (b5[i+1] - b5[i])/DIq_const[i];
        Da6[i] = (a6[i+1] - a6[i])/DIq_const[i];
        Db6[i] = (b6[i+1] - b6[i])/DIq_const[i];
        Da7[i] = (a7[i+1] - a7[i])/DIq_const[i];
        Db7[i] = (b7[i+1] - b7[i])/DIq_const[i];
        Da8[i] = (a8[i+1] - a8[i])/DIq_const[i];
        Db8[i] = (b8[i+1] - b8[i])/DIq_const[i];
        Dw [i] = (w [i+1] - w [i])/DIq_const[i];
    }
}
//-1   0    1    2    3    4    5     6    7    8   9   10  11  12   13  14  15  16  17 
//   0    1    2    3    4    5     6    7    8   9   10  11  12   13  14  15  16  17 
//  -3.0 -2.5 -2.0 -1.5 -1.0 -0.75 -0.5 -0.3 -0.1 0.1 0.3 0.5 0.75 1.0 1.5 2.0 2.5 3.0
float HFI_angle_Rec3(MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    float t_a0, t_a1, t_a2, t_a3, t_a4, t_a5, t_a6, t_a7, t_a8;
    float t_b1, t_b2, t_b3, t_b4, t_b5, t_b6, t_b7, t_b8, t_w;

    int8_t I_temp = 0;

    if(MRT_Inf->Iq_Ave < Iq_const[8]){ // Iq < -0.1A
        if(MRT_Inf->Iq_Ave < Iq_const[3]){ // Iq < -1.5A
            if(MRT_Inf->Iq_Ave < Iq_const[1]){ // Iq < -2.5A
                if(MRT_Inf->Iq_Ave < Iq_const[0]){ // Iq < -3.0A
                    I_temp = -1;
                }
                else{ // -2.5A < Iq < -3.0A
                    I_temp = 0;
                }
            }
            else{ // -2.5A < Iq < -1.5A
                if(MRT_Inf->Iq_Ave < Iq_const[2]){ // -2.5A < Iq < -2.0A
                    I_temp = 1;
                }
                else{ // -2.0A < Iq < -1.5A
                    I_temp = 2;
                }
            }
        }
        else{ // -1.5A < Iq < -0.1A
            if(MRT_Inf->Iq_Ave < Iq_const[6]){ // -1.5A < Iq < -0.5A
                if(MRT_Inf->Iq_Ave < Iq_const[4]){ // -1.5A < Iq < -1.0A
                    I_temp = 3;
                }
                else if(MRT_Inf->Iq_Ave < Iq_const[5]){ // -1.0A < Iq < -0.75A
                    I_temp = 4;
                }
                else{ // -0.75A < Iq < -0.5A
                    I_temp = 5;
                }
            }
            else{ // -0.5A < Iq < -0.1A
                if(MRT_Inf->Iq_Ave < Iq_const[7]){ // -0.5A < Iq < -0.3A
                    I_temp = 6;
                }
                else{ // -0.3A < Iq < -0.1A
                    I_temp = 7;
                }
            }
        }
    }
    else{ // -0.1A < Iq 
        if(MRT_Inf->Iq_Ave < Iq_const[13]){ // -0.1A < Iq < 1.0A
            if(MRT_Inf->Iq_Ave < Iq_const[11]){ // -0.1A < Iq < 0.5A
                if(MRT_Inf->Iq_Ave < Iq_const[9]){ // -0.1A < Iq < 0.1A
                    I_temp = 8;
                }
                else if(MRT_Inf->Iq_Ave < Iq_const[10]){ // 0.1A < Iq < 0.3A
                    I_temp = 9;
                }
                else{ // 0.3A < Iq < 0.5A
                    I_temp = 10;
                }
            }
            else{ // 0.5A < Iq < 1.0A
                if(MRT_Inf->Iq_Ave < Iq_const[12]){ // 0.5A < Iq < 0.75A
                    I_temp = 11;
                }
                else{  // 0.75A < Iq < 1.0A
                    I_temp = 12;
                }
            }
        }
        else{ // 1.0A < Iq
            if(MRT_Inf->Iq_Ave < Iq_const[15]){ // 1.0A < Iq < 2.0A
                if(MRT_Inf->Iq_Ave < Iq_const[14]){ // 1.0A < Iq < 1.5A
                    I_temp = 13;
                }
                else{ // 1.5A < Iq < 2.0A
                    I_temp = 14;
                }
            }
            else{ // 2.0A < Iq
                if(MRT_Inf->Iq_Ave < Iq_const[16]){ // 2.0A < Iq < 2.5A
                    I_temp = 15;
                }
                else if(MRT_Inf->Iq_Ave < Iq_const[17]){ // 2.5A < Iq < 3.0A
                    I_temp = 16;
                }
                else{  // 3.0A < Iq
                    I_temp = 17;
                }
            }
        }
    }

    if(I_temp == -1){
        t_a0 = a0[0];
        t_a1 = a1[0];
        t_a2 = a2[0];
        t_a3 = a3[0];
        t_a4 = a4[0];
        t_a5 = a5[0];
        t_a6 = a6[0];
        t_a7 = a7[0];
        t_a8 = a8[0];
        t_b1 = b1[0];
        t_b2 = b2[0];
        t_b3 = b3[0];
        t_b4 = b4[0];
        t_b5 = b5[0];
        t_b6 = b6[0];
        t_b7 = b7[0];
        t_b8 = b8[0];
        t_w  = w[0];
    }
    else if(I_temp == 17){
        t_a0 = a0[17];
        t_a1 = a1[17];
        t_b1 = b1[17];
        t_a2 = a2[17];
        t_b2 = b2[17];
        t_a3 = a3[17];
        t_b3 = b3[17];
        t_a4 = a4[17];
        t_b4 = b4[17];
        t_a5 = a5[17];
        t_b5 = b5[17];
        t_a6 = a6[17];
        t_b6 = b6[17];
        t_a7 = a7[17];
        t_b7 = b7[17];
        t_a8 = a8[17];
        t_b8 = b8[17];
        t_w  = w [17];
    }
    else {
        t_a0 = Da0[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a0[I_temp];
        t_a1 = Da1[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a1[I_temp];
        t_b1 = Db1[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + b1[I_temp];
        t_a2 = Da2[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a2[I_temp];
        t_b2 = Db2[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + b2[I_temp];
        t_a3 = Da3[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a3[I_temp];
        t_b3 = Db3[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + b3[I_temp];
        t_a4 = Da4[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a4[I_temp];
        t_b4 = Db4[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + b4[I_temp];
        t_a5 = Da5[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a5[I_temp];
        t_b5 = Db5[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + b5[I_temp];
        t_a6 = Da6[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a6[I_temp];
        t_b6 = Db6[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + b6[I_temp];
        t_a7 = Da7[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a7[I_temp];
        t_b7 = Db7[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + b7[I_temp];
        t_a8 = Da8[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + a8[I_temp];
        t_b8 = Db8[I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + b8[I_temp];
        t_w  = Dw [I_temp] * (MRT_Inf->Iq_Ave - Iq_const[I_temp]) + w [I_temp];
    }

    float T1, T2, T3, T4, T5, T6, T7, T8;

    T1 = 1.0f * t_w * HFI->ThetaE;
    T2 = 2.0f * t_w * HFI->ThetaE;
    T3 = 3.0f * t_w * HFI->ThetaE;
    T4 = 4.0f * t_w * HFI->ThetaE;
    T5 = 5.0f * t_w * HFI->ThetaE;
    T6 = 6.0f * t_w * HFI->ThetaE;
    T7 = 7.0f * t_w * HFI->ThetaE;
    T8 = 8.0f * t_w * HFI->ThetaE;

    float s1, s2, s3, s4, s5, s6, s7, s8;
    float c1, c2, c3, c4, c5, c6, c7, c8;

    s1 = arm_sin_f32(T1); c1 = arm_cos_f32(T1);
    s2 = arm_sin_f32(T2); c2 = arm_cos_f32(T2);
    s3 = arm_sin_f32(T3); c3 = arm_cos_f32(T3);
    s4 = arm_sin_f32(T4); c4 = arm_cos_f32(T4);
    s5 = arm_sin_f32(T5); c5 = arm_cos_f32(T5);
    s6 = arm_sin_f32(T6); c6 = arm_cos_f32(T6);
    s7 = arm_sin_f32(T7); c7 = arm_cos_f32(T7);
    s8 = arm_sin_f32(T8); c8 = arm_cos_f32(T8);

    float f1 = t_a1 * c1 + t_b1 * s1;
    float f2 = t_a2 * c2 + t_b2 * s2;
    float f3 = t_a3 * c3 + t_b3 * s3;
    float f4 = t_a4 * c4 + t_b4 * s4;
    float f5 = t_a5 * c5 + t_b5 * s5;
    float f6 = t_a6 * c6 + t_b6 * s6;
    float f7 = t_a7 * c7 + t_b7 * s7;
    float f8 = t_a8 * c8 + t_b8 * s8;

    float HFI_ThetaE_Rec_temp = HFI->ThetaE + t_a0 + f1 + f2 + f3 + f4 + f5 + f6 + f7 + f8;

    if(HFI_ThetaE_Rec_temp < 0)
        HFI_ThetaE_Rec_temp += 2 * PI;
    HFI_ThetaE_Rec_temp = fmodf(HFI_ThetaE_Rec_temp, 2 * PI);

    return HFI_ThetaE_Rec_temp;
}

