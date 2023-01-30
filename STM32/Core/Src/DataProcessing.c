#include "DataProcessing.h"

void LPF(float* Uo, float Ui, float Fs, float Wc){
    *Uo += Wc / Fs * (Ui - *Uo);
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

float PI_Control(PI_str* pPI){
    uint8_t ui_flag = !(((pPI->Out_temp > pPI->Max) || (pPI->Out_temp < -pPI->Max)) && (pPI->Out_temp * pPI->Error >= 0));
    
    pPI->up = pPI->Kp * pPI->Error;
    pPI->ui = pPI->ui + pPI->Ki * pPI->Error * ui_flag;
    
    pPI->Out_temp = pPI->up + pPI->ui;
    
    float PIout = 0;
    
    if(pPI->Out_temp > pPI->Max)
        PIout = pPI->Max;
    else if(pPI->Out_temp < -pPI->Max)
        PIout = -pPI->Max;
    else 
        PIout = pPI->Out_temp;
    
    return PIout;
}

float sawtooth(float angle, float width){
    float output;
    float k1 = 2.0f/width;
    float k2 = -2.0f/(1 - width);
    float pos = angle / (2 * PI);

    pos = fmodf(pos, 1);

    if(pos < width){
        output = k1 * pos - 1;
    }
    else {
        output = k2 * (pos - 1) - 1;
    }

    return output;
}
