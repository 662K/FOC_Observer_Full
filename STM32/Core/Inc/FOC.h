#ifndef __FOC_H
#define __FOC_H

#include "main.h"

#define CurLoopStop_Threshold 0.03f
#define SpdLoopStop_Threshold 0.5f
#define HFIIhErr_Threshold 0.02f
#define HFIThetaEErr_Threshold 0.001f
#define HFIStop_Threshold 3.0f

void FOC_Mode_Select(PI_str* D_PI, PI_str* Q_PI, PI_str* Spd_PI, ControlCommand_str* CtrlCom, MotorParameter_str* MotorParameter, MotorRealTimeInformation_str* MRT_Inf, SlidingModeObserver_str* SMO, HighFrequencyInjection_str* HFI);
#endif
