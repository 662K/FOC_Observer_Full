#include "DataSampling.h"

float GetTheta(int32_t Theta){
    return 2 * PI * Theta / (1 << 20);
}

float GetCur(int32_t Cur){
    return 5.0 * (Cur - 2048) / (1 << 11);
}

float GetThetaE(float ThetaE, uint8_t Np){
    return fmodf((ThetaE * (float)Np), (2.0f * PI));
}

void GetSpd(float Theta, float* Theta_Pre, uint8_t Spd_Tick, float* Speed, float SpdTs, int* Start_Flag){
    if(Spd_Tick == 0){
        if(*Start_Flag != 1){
            *Start_Flag = 1;
            *Theta_Pre = Theta;
        }
        else{
            float Speed_temp = Theta - *Theta_Pre;
            if(Speed_temp > 0.314)
                Speed_temp = Speed_temp - 2*PI;
            else if(Speed_temp < -0.314)
                Speed_temp = Speed_temp + 2*PI;
            else
                Speed_temp = Speed_temp;
            
            *Speed = Speed_temp / SpdTs;
            *Theta_Pre = Theta;
        }
    }
}

void CtrlComFilter(float *Target, float CtrlCom, float TickAdd){
    if(*Target < CtrlCom){
        if(*Target + TickAdd > CtrlCom){
            *Target = CtrlCom;
        }
        else{
            *Target += TickAdd;
        }
    }
    else if(*Target > CtrlCom){
        if(*Target - TickAdd < CtrlCom){
            *Target = CtrlCom;
        }
        else{
            *Target -= TickAdd;
        }
    }          
}
