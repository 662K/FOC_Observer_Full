#include "FOC.h"
#include "FOCSub.h"
#include "Sensorless.h"
#include "DataProcessing.h"

void VolLoop_Start(ControlCommand_str* CtrlCom){
    CtrlCom->Uq_Target = 0;
    CtrlCom->Status_Flag = RUN;
}

void VolLoop_Mode(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    
    MRT_Inf->Ud = 0;
    MRT_Inf->Uq = CtrlCom->Uq_Target;
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void VolLoop_Stop(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    if(CtrlCom->Uq != 0){
        CtrlCom->Uq_Target = 0;
        VolLoop_Mode(CtrlCom, MotorParameter, MRT_Inf);
    }
    else{
        CtrlCom->Status_Flag = START;
    }
}

void CurLoop_Start(ControlCommand_str* CtrlCom){
    CtrlCom->Id_Target = 0;
    CtrlCom->Status_Flag = RUN;
}

void CurLoop_Mode(PI_str* D_PI, PI_str* Q_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    
    CtrlCom->Id = CtrlCom->Id_Target;
    CtrlCom->Iq = 0;

    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void CurLoop_Stop(PI_str* D_PI, PI_str* Q_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    if((MRT_Inf->Id < CurLoopStop_Threshold) && (MRT_Inf->Id > -CurLoopStop_Threshold) && (MRT_Inf->Iq < CurLoopStop_Threshold) && (MRT_Inf->Iq > -CurLoopStop_Threshold)){
        CtrlCom->Stop_cnt++;
    }
    else{
        CtrlCom->Stop_cnt = 0;
    }
    
    if(CtrlCom->Stop_cnt < 500){
        CtrlCom->Id_Target = 0;
        CurLoop_Mode(D_PI, Q_PI, CtrlCom, MotorParameter, MRT_Inf);
    }
    else{
        CtrlCom->Stop_cnt = 0;
        CtrlCom->Status_Flag = START;
    }
}

void SpdLoop_Start(ControlCommand_str* CtrlCom){
    CtrlCom->Spd_Target_rpm = 0;
    CtrlCom->Spd_Target = 0;
    CtrlCom->Status_Flag = RUN;
}

void SpdLoop_Mode(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);
    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &MRT_Inf->Ix, &MRT_Inf->Iy);
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    
    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        Spd_PI->Error = CtrlCom->Spd - MRT_Inf->Spd;
        CtrlCom->Iq = PI_Control(Spd_PI);
    }

    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    InvClarke(MRT_Inf->Ux, MRT_Inf->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);
}

void SpdLoop_Stop(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf){
    if((MRT_Inf->Spd < SpdLoopStop_Threshold) && (MRT_Inf->Spd > -SpdLoopStop_Threshold)){
        CtrlCom->Stop_cnt++;
    }
    else{
        CtrlCom->Stop_cnt = 0;
    }

    if(CtrlCom->Stop_cnt < 500){
        CtrlCom->Spd_Target_rpm = 0;
        CtrlCom->Spd_Target = 0;
        SpdLoop_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf);
    }
    else{
        CtrlCom->Stop_cnt = 0;
        CtrlCom->Status_Flag = START;
    }
}

void HFI_Start(ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    switch (HFI->Start_Status){
    case IH_INIT:
        if((HFI->Ih_Err > -HFIIhErr_Threshold) && (HFI->Ih_Err < HFIIhErr_Threshold)){
            HFI->status_cnt++;
        }
        else{
            HFI->status_cnt = 0;
        }
        
        if(HFI->status_cnt >= 500){
            HFI->Start_Status = THETAE_INIT;
            HFI->status_cnt = 0;
        }

        Ih_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case THETAE_INIT:
        if((HFI->ThetaE_Err > -HFIThetaEErr_Threshold) && (HFI->ThetaE_Err < HFIThetaEErr_Threshold)){
            HFI->status_cnt++;
        }
        else{
            HFI->status_cnt = 0;
        }
        
        if(HFI->status_cnt >= 500){
            HFI->Start_Status = POLE_INIT;
            HFI->status_cnt = 0;
        }

        ThetaE_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case POLE_INIT:
        if(HFI->Pulse_cnt == 100){
            if(HFI->Ipulse_Max > -HFI->Ipulse_Min){
                HFI->Start_Status = POLE_N_WAIT;
                HFI->status_cnt = 0;
            }
            else if(HFI->Ipulse_Max < -HFI->Ipulse_Min){
                HFI->Start_Status = POLE_S_WAIT;
                HFI->status_cnt = 0;
            }
        }
        
        Pole_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case POLE_N_WAIT:
        if(HFI->Pulse_cnt == 100){
            if(HFI->Ipulse_Max > -HFI->Ipulse_Min){
                HFI->status_cnt++;
            }
            else{
                HFI->Start_Status = POLE_INIT;
                HFI->status_cnt = 0;
            }
            
            if(HFI->status_cnt >= HFI->Pulse_Max_cnt){
                HFI->Start_Status = HFI_WAIT;
                HFI->status_cnt = 0;
            }
        }

        Pole_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case POLE_S_WAIT:
        if(HFI->Pulse_cnt == 100){
            if(HFI->Ipulse_Max < -HFI->Ipulse_Min){
                HFI->status_cnt++;
            }
            else{
                HFI->Start_Status = POLE_INIT;
                HFI->status_cnt = 0;
            }
            
            if(HFI->status_cnt >= HFI->Pulse_Max_cnt){
                HFI->Start_Status = HFI_WAIT;
                HFI->ThetaE = fmodf(HFI->ThetaE + PI, 2 * PI);
                HFI->status_cnt = 0;
            }
        }

        Pole_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    case HFI_WAIT:
        if(HFI->status_cnt >= 500){
            CtrlCom->Status_Flag = RUN;
            HFI->Run_Status = HFI_WORK;
            HFI->status_cnt = 0;
        }
        else{
            HFI->status_cnt++;
        }
        ThetaE_Init(CtrlCom, MotorParameter, MRT_Inf, HFI);
        break;
    }
}

void HFI_Mode(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    float HFI_SpdE = 0;
    float HFI_SpdE_Rec = 0;
    float HFI_Ih = 0;
    float HFI_ThetaE_temp = 0;
    float HFI_ThetaE_Rec_temp = 0;
    
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
    
    Cordic(fmodf(HFI->Rec.b1*HFI_ThetaE_temp + HFI->Rec.c1, 2*PI), &HFI->SinThetaE_Rec, &HFI->CosThetaE_Rec);
    HFI_ThetaE_Rec_temp = HFI_ThetaE_temp + HFI->Rec.a1*HFI->SinThetaE_Rec + HFI->Rec.d1 + HFI->Rec.d2 * MRT_Inf->Iq_Ave / 2.7f;
    if(HFI_ThetaE_Rec_temp < 0)
        HFI_ThetaE_Rec_temp += 2 * PI;
    HFI->ThetaE_Rec = fmodf(HFI_ThetaE_Rec_temp, 2 * PI);
    
    Cordic(HFI->ThetaE_Rec, &HFI->SinTheta_Rec, &HFI->CosTheta_Rec);
    
    HFI_SpdE_Rec = HFI->ThetaE_Rec - HFI->ThetaE_Rec_temp;
    if(HFI_SpdE_Rec > PI)
        HFI_SpdE_Rec -= 2*PI;
    else if(HFI_SpdE_Rec < -PI)
        HFI_SpdE_Rec += 2*PI;
    HFI_SpdE_Rec = HFI_SpdE_Rec / CtrlCom->CurTs;
    LPF(&(HFI->SpdE_Rec), HFI_SpdE_Rec, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, HFI->SinTheta_Rec, HFI->CosTheta_Rec);
    LPF(&(MRT_Inf->Iq_Ave), MRT_Inf->Iq, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
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

void HFI_Stop(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    if((HFI->Spd_Rec < HFIStop_Threshold) && (HFI->Spd_Rec > -HFIStop_Threshold)){
        CtrlCom->Stop_cnt++;
    }
    else{
        CtrlCom->Stop_cnt = 0;
    }

    if(CtrlCom->Stop_cnt < 500){
        CtrlCom->Spd_Target_rpm = 0;
        CtrlCom->Spd_Target = 0;
        HFI_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, HFI);
    }
    else{
        CtrlCom->Stop_cnt = 0;
        CtrlCom->Status_Flag = START;
        HFI->Start_Status = IH_INIT;
    }
}

void HFIwithSMO_Mode(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    switch (HFI->Run_Status){
    case HFI_WORK:
        HFI_Work(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO, HFI);

        if(SMO->EMF_Rms_Test > SMO->Switch_EMF){
            HFI->Run_Status = HFI_SMO_MIX;
            SMO->EMF_Dir = (SMO->SpdE > 0)?(1):(-1);
            HFI->Run_Status_temp = HFI_WORK;
        }
        break;
    case HFI_SMO_MIX:
        HFI_SMO_Mix_Work(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO, HFI);

        if(SMO->EMF_Rms_Test > SMO->Switch_EMF * 2){
            HFI->Run_Status = SMO_WORK;
        }
        else if(SMO->EMF_Rms_Test < SMO->Switch_EMF){
            HFI->Run_Status = HFI_WORK;
        }
        break;
    case SMO_WORK:
        SMO_Work(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO, HFI);

        if(SMO->EMF_Rms_Test < SMO->Switch_EMF * 2){
            HFI->Run_Status = HFI_SMO_MIX;
            // Cordic(fmodf(HFI->Rec.b1*HFI_ThetaE_temp + HFI->Rec.c1, 2*PI), &HFI->SinThetaE_Rec, &HFI->CosThetaE_Rec);
            // HFI_ThetaE_Rec_temp = HFI_ThetaE_temp + HFI->Rec.a1*HFI->SinThetaE_Rec + HFI->Rec.d1 + HFI->Rec.d2 * MRT_Inf->Iq_Ave / 2.7f;
            HFI->ThetaE = SMO->ThetaE;
            HFI->SpdE_Rec = SMO->SpdE;
            HFI->Run_Status_temp = SMO_WORK;
        }
        break;
    }
}

void HFIwithSMO_Stop(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    if((HFI->Spd_Rec < HFIStop_Threshold) && (HFI->Spd_Rec > -HFIStop_Threshold)){
        CtrlCom->Stop_cnt++;
    }
    else{
        CtrlCom->Stop_cnt = 0;
    }

    if(CtrlCom->Stop_cnt < 500){
        CtrlCom->Spd_Target_rpm = 0;
        CtrlCom->Spd_Target = 0;
        HFIwithSMO_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO, HFI);
    }
    else{
        CtrlCom->Stop_cnt = 0;
        CtrlCom->Status_Flag = START;
        HFI->Start_Status = IH_INIT;
        HFI->Run_Status = HFI_WORK;
        SMO->Dir = 0;
    }
}

void SpdwithHFI_Mode(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, HighFrequencyInjection_str* HFI){
    float HFI_Ih = 0;
    float HFI_SpdE = 0;
    float HFI_ThetaE_temp = 0;
    float HFI_ThetaE_Rec_temp = 0;

    Cordic(MRT_Inf->ThetaE, &MRT_Inf->SinTheta, &MRT_Inf->CosTheta);

    arm_clarke_f32(MRT_Inf->Ia, MRT_Inf->Ib, &HFI->Ix, &HFI->Iy);

    MRT_Inf->Ix = (HFI->Ix + HFI->Ix_temp) / 2;
    MRT_Inf->Iy = (HFI->Iy + HFI->Iy_temp) / 2;
    HFI->Ixh = (HFI->Ix - HFI->Ix_temp) / 2 * HFI->Dir;
    HFI->Iyh = (HFI->Iy - HFI->Iy_temp) / 2 * HFI->Dir;

    arm_sqrt_f32(HFI->Ixh * HFI->Ixh + HFI->Iyh * HFI->Iyh, &HFI_Ih);
    LPF(&HFI->Ih, HFI_Ih, CtrlCom->CurFs, HFI->Ih_LPF_wc);
    
    Cordic(HFI->ThetaE, &HFI->SinTheta, &HFI->CosTheta);

    HFI->SpdE_PI.Error = (HFI->Ixh * HFI->SinTheta - HFI->Iyh * HFI->CosTheta) / 0.266f;
    HFI_SpdE = PI_Control(&HFI->SpdE_PI);
    HFI_ThetaE_temp = HFI->ThetaE + HFI_SpdE * CtrlCom->CurTs;
    if(HFI_ThetaE_temp < 0)
        HFI_ThetaE_temp += 2 * PI;
    HFI->ThetaE = fmodf(HFI_ThetaE_temp, 2 * PI);

    HFI->ThetaE_Err = MRT_Inf->ThetaE - HFI->ThetaE;
    if(HFI->ThetaE_Err > PI)
        HFI->ThetaE_Err -= 2*PI;
    else if(HFI->ThetaE_Err < -PI)
        HFI->ThetaE_Err += 2*PI;

    Cordic(fmodf(HFI->Rec.b1*HFI_ThetaE_temp + HFI->Rec.c1, 2*PI), &HFI->SinThetaE_Rec, &HFI->CosThetaE_Rec);
    HFI_ThetaE_Rec_temp = HFI_ThetaE_temp + HFI->Rec.a1*HFI->SinThetaE_Rec + HFI->Rec.d1 + HFI->Rec.d2 * MRT_Inf->Iq_Ave / 2.7f;
    if(HFI_ThetaE_Rec_temp < 0)
        HFI_ThetaE_Rec_temp += 2 * PI;
    HFI->ThetaE_Rec = fmodf(HFI_ThetaE_Rec_temp, 2 * PI);

    HFI->ThetaE_Err2 = MRT_Inf->ThetaE - HFI->ThetaE_Rec;
    if(HFI->ThetaE_Err2 > PI)
        HFI->ThetaE_Err2 -= 2*PI;
    else if(HFI->ThetaE_Err2 < -PI)
        HFI->ThetaE_Err2 += 2*PI;

    HFI->ThetaE_Rec2 = HFI_angle_Rec3(MRT_Inf, HFI);

    arm_park_f32(MRT_Inf->Ix, MRT_Inf->Iy, &MRT_Inf->Id, &MRT_Inf->Iq, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    LPF(&(MRT_Inf->Iq_Ave), MRT_Inf->Iq, CtrlCom->CurFs, HFI->Spd_LPF_wc);
    
    if(CtrlCom->Spd_Tick == 0){
        CtrlCom->Id = 0;
        Spd_PI->Error = CtrlCom->Spd - MRT_Inf->Spd;
        CtrlCom->Iq = PI_Control(Spd_PI);
    }

    D_PI->Error = CtrlCom->Id - MRT_Inf->Id;
    MRT_Inf->Ud = PI_Control(D_PI);
    Q_PI->Error = CtrlCom->Iq - MRT_Inf->Iq;
    MRT_Inf->Uq = PI_Control(Q_PI);

    HFI->Dir = (HFI->Dir == 1)?(-1):(1);
    HFI->Vdh = HFI->Dir * HFI->Vh;
    
    arm_inv_park_f32(MRT_Inf->Ud, MRT_Inf->Uq, &MRT_Inf->Ux, &MRT_Inf->Uy, MRT_Inf->SinTheta, MRT_Inf->CosTheta);
    arm_inv_park_f32(HFI->Vdh, 0, &HFI->Ux, &HFI->Uy, HFI->SinTheta, HFI->CosTheta);
    InvClarke(MRT_Inf->Ux + HFI->Ux, MRT_Inf->Uy + HFI->Uy, &MRT_Inf->U1, &MRT_Inf->U2, &MRT_Inf->U3);
    GetSector(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, &MRT_Inf->Sector);
    GetCCR(MRT_Inf->U1, MRT_Inf->U2, MRT_Inf->U3, MRT_Inf->Sector, MRT_Inf->Uac, &MRT_Inf->CCRa, &MRT_Inf->CCRb, &MRT_Inf->CCRc);

    HFI->Ix_temp = HFI->Ix;
    HFI->Iy_temp = HFI->Iy;
}

void SpdwithHFI_Stop(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    if((MRT_Inf->Spd < SpdLoopStop_Threshold) && (MRT_Inf->Spd > -SpdLoopStop_Threshold)){
        CtrlCom->Stop_cnt++;
    }
    else{
        CtrlCom->Stop_cnt = 0;
    }

    if(CtrlCom->Stop_cnt < 500){
        CtrlCom->Spd_Target_rpm = 0;
        CtrlCom->Spd_Target = 0;
        SpdwithHFI_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, HFI);
    }
    else{
        CtrlCom->Stop_cnt = 0;
        CtrlCom->Status_Flag = START;
    }
}

void FOC_Mode_Select(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI){
    switch (CtrlCom->Status_Flag){
    case START:
        CtrlCom->Mode_last = CtrlCom->Mode;

        switch(CtrlCom->Mode){
        case VolLoop:
            VolLoop_Start(CtrlCom);
            break;
        case CurLoop:
            CurLoop_Start(CtrlCom);
            break;
        case SpdLoop:
            SpdLoop_Start(CtrlCom);
            break;
        case HFI_Sensorless:
            HFI_Start(CtrlCom, MotorParameter, MRT_Inf, HFI);
            break;
        case HFIwithSMO_Sensorless:
            HFI_Start(CtrlCom, MotorParameter, MRT_Inf, HFI);
            break;
        case SpdwithHFI:
            HFI_Start(CtrlCom, MotorParameter, MRT_Inf, HFI);
            break;
        }
        break;
    case RUN:
        switch(CtrlCom->Mode){
        case VolLoop:
            VolLoop_Mode(CtrlCom, MotorParameter, MRT_Inf);
            break;
        case CurLoop:
            CurLoop_Mode(D_PI, Q_PI, CtrlCom, MotorParameter, MRT_Inf);
            break;
        case SpdLoop:
            SpdLoop_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf);
            break;
        case HFI_Sensorless:
            HFI_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, HFI);
            break;
        case HFIwithSMO_Sensorless:
            HFIwithSMO_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO, HFI);
            break;
        case SpdwithHFI:
            SpdwithHFI_Mode(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, HFI);
            break;
        }
        break;
    case STOP:
        switch(CtrlCom->Mode_last){
        case VolLoop:
            VolLoop_Stop(CtrlCom, MotorParameter, MRT_Inf);
            break;
        case CurLoop:
            CurLoop_Stop(D_PI, Q_PI, CtrlCom, MotorParameter, MRT_Inf);
            break;
        case SpdLoop:
            SpdLoop_Stop(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf);
            break;
        case HFI_Sensorless:
            HFI_Stop(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, HFI);
            break;
        case HFIwithSMO_Sensorless:
            HFIwithSMO_Stop(D_PI, Q_PI, Spd_PI, CtrlCom, MotorParameter, MRT_Inf, SMO, HFI);
            break;
        }
        break;
    }
}
