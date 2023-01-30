#define S_FUNCTION_NAME  FOCSF
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "FOC.h"
#include "DataSampling.h"

PI_str D_PI  = {0};
PI_str Q_PI  = {0};
PI_str Spd_PI  = {0};
ControlCommand_str CtrlCom = {0};
MotorParameter_str MotorParameter = {0};
MotorRealTimeInformation_str MRT_Inf = {0};
SlidingModeObserver_str SMO = {0};

#define ParameterNum 8
#define InputPortNum 4
#define OutputPortNum 5
/* 模块初始化函数 */
static void mdlInitializeSizes(SimStruct *S)
{
    memset(&D_PI, 0, sizeof(D_PI));
    memset(&Q_PI, 0, sizeof(Q_PI));
    memset(&Spd_PI, 0, sizeof(Spd_PI));
    memset(&CtrlCom, 0, sizeof(CtrlCom));
    memset(&MotorParameter, 0, sizeof(MotorParameter));
    memset(&MRT_Inf, 0, sizeof(MRT_Inf));
    memset(&SMO, 0, sizeof(SMO));

    /* 设置参数数量 */
    ssSetNumSFcnParams(S, ParameterNum);
    for(int i = 0; i < ParameterNum; i++)
        ssSetSFcnParamTunable(S, i, 1);

    /* 设置输入端口数量 */
    if (!ssSetNumInputPorts(S, InputPortNum)) return;

    /* 配置输入端口 */
    int InputPortWidth[InputPortNum] = {1, 3, 11, 10};
    for(int i = 0; i < InputPortNum; i++){
        ssSetInputPortDataType(S, i, SS_DOUBLE);   
        ssSetInputPortDirectFeedThrough(S, i, TRUE);
        ssSetInputPortRequiredContiguous(S, i, TRUE);
        ssSetInputPortWidth(S, i, InputPortWidth[i]);
    }

    /* 设置输出端口数量 */
    if (!ssSetNumOutputPorts(S, OutputPortNum)) return;

    /* 配置输出端口 */
    int OutputPortWidth[OutputPortNum] = {16, 2, 2, 1, 9};
    for(int i = 0; i < OutputPortNum; i++){
        ssSetOutputPortDataType(S, i, SS_DOUBLE);
        ssSetOutputPortWidth(S, i, OutputPortWidth[i]);
    }
}

/* 模块采样时间初始化函敿 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
	/* 设置采样时间为从连接的端口继承采样时间 */
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
	/* 设置采样偏移时间 */
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
}

/* 模块输出函数 */
static void mdlOutputs(SimStruct *S, int_T tid){
    /* 获取参数/输入/输出的数据 */
    real_T* iTheta     = (real_T*) ssGetInputPortSignal(S, 0);
    real_T* iIabc      = (real_T*) ssGetInputPortSignal(S, 1);
    real_T* iCtrlParam = (real_T*) ssGetInputPortSignal(S, 2);
    real_T* iSMOParam  = (real_T*) ssGetInputPortSignal(S, 3);

    real_T* Np  = (real_T*) ssGetRunTimeParamInfo(S, 0)->data;
    real_T* Udc = (real_T*) ssGetRunTimeParamInfo(S, 1)->data;
    real_T* Ls  = (real_T*) ssGetRunTimeParamInfo(S, 2)->data;
    real_T* Rs  = (real_T*) ssGetRunTimeParamInfo(S, 3)->data;
    real_T* Kt  = (real_T*) ssGetRunTimeParamInfo(S, 4)->data;
    real_T* J   = (real_T*) ssGetRunTimeParamInfo(S, 5)->data;
    real_T* Ld  = (real_T*) ssGetRunTimeParamInfo(S, 6)->data;
    real_T* Lq  = (real_T*) ssGetRunTimeParamInfo(S, 7)->data;

    real_T* oFOC = (real_T*) ssGetOutputPortSignal(S, 0);
    real_T* oIdq = (real_T*) ssGetOutputPortSignal(S, 1);
    real_T* oUdq = (real_T*) ssGetOutputPortSignal(S, 2);
    real_T* oSpd = (real_T*) ssGetOutputPortSignal(S, 3);
    real_T* oSMO = (real_T*) ssGetOutputPortSignal(S, 4);

    D_PI.Kp    = iCtrlParam[0];
    D_PI.Ki    = iCtrlParam[1];
    D_PI.Max   = iCtrlParam[2];
    
    Q_PI.Kp    = iCtrlParam[0];
    Q_PI.Ki    = iCtrlParam[1];
    Q_PI.Max   = iCtrlParam[2];
    
    Spd_PI.Kp  = iCtrlParam[3];
    Spd_PI.Ki  = iCtrlParam[4];
    Spd_PI.Max = iCtrlParam[5];

    MotorParameter.Np = (uint8_t)(*Np);
    MotorParameter.Ls = *Ls;
    MotorParameter.Rs = *Rs;
    MotorParameter.Kt = *Kt;
    MotorParameter.J  = *J;
    MotorParameter.Flux = MotorParameter.Kt / 1.5 / MotorParameter.Np;
    MotorParameter.Ld = *Ld;
    MotorParameter.Lq = *Lq;

    CtrlCom.Mode  = (uint8_t)iCtrlParam[8];
    CtrlCom.CurTs = iCtrlParam[6];
    CtrlCom.SpdTs = iCtrlParam[7];
    CtrlCom.CurFs = 1.0 / CtrlCom.CurTs;
    CtrlCom.SpdFs = 1.0 / CtrlCom.SpdTs;

    MRT_Inf.Udc   = *Udc;
    MRT_Inf.Theta = GetTheta((int32_t)iTheta[0]);
    MRT_Inf.Ia = GetCur((int32_t)iIabc[0]);
    MRT_Inf.Ic = GetCur((int32_t)iIabc[2]);

    SMO.h1 = iSMOParam[0];
    SMO.h2 = iSMOParam[1];
    SMO.E1 = iSMOParam[2];
    SMO.EMF_LPF_wc = iSMOParam[3];
    SMO.m = iSMOParam[4];
    SMO.Spd_LPF_wc = iSMOParam[5];
    SMO.Switch_Spd = iSMOParam[6];
    SMO.Switch_EMF = SMO.Switch_Spd * MotorParameter.Np * MotorParameter.Flux;
    SMO.Theta_PLL_wn = iSMOParam[7];
    SMO.Theta_PLL_zeta = iSMOParam[8];
    SMO.Theta_PLL_we = iSMOParam[9];

    SMO.NPLL_PI.Kp = 2 * SMO.Theta_PLL_zeta * SMO.Theta_PLL_wn;
    SMO.NPLL_PI.Ki = SMO.Theta_PLL_wn * SMO.Theta_PLL_wn * CtrlCom.CurTs;
    SMO.NPLL_PI.Max = 2 * PI * 200 * MotorParameter.Np;

    SMO.PLL_PI.Kp = 2 * SMO.Theta_PLL_zeta * SMO.Theta_PLL_wn / MotorParameter.Flux / SMO.Theta_PLL_we;
    SMO.PLL_PI.Ki = SMO.Theta_PLL_wn * SMO.Theta_PLL_wn / MotorParameter.Flux / SMO.Theta_PLL_we * CtrlCom.CurTs;
    SMO.PLL_PI.Max = 2 * PI * 200 * MotorParameter.Np;

    CtrlComFilter(&CtrlCom.Spd, iCtrlParam[9], iCtrlParam[10]);
    
    /* 调用函数接口 */
    FOC(&D_PI, &Q_PI, &Spd_PI, &CtrlCom, &MotorParameter, &MRT_Inf, &SMO);

    oFOC[0]  = MRT_Inf.SinTheta;
    oFOC[1]  = MRT_Inf.CosTheta;
    oFOC[2]  = MRT_Inf.Ux;
    oFOC[3]  = MRT_Inf.Uy;
    oFOC[4]  = MRT_Inf.U1;
    oFOC[5]  = MRT_Inf.U2;
    oFOC[6]  = MRT_Inf.U3;
    oFOC[7]  = MRT_Inf.Sector;
    oFOC[8]  = MRT_Inf.CCRa;
    oFOC[9]  = MRT_Inf.CCRb;
    oFOC[10] = MRT_Inf.CCRc;
    oFOC[11] = MRT_Inf.Ix;
    oFOC[12] = MRT_Inf.Iy;
    oFOC[13] = MRT_Inf.Ex;
    oFOC[14] = MRT_Inf.Ey;
    oFOC[15] = MRT_Inf.ThetaE;

    oIdq[0] = MRT_Inf.Id;
    oIdq[1] = MRT_Inf.Iq;

    oUdq[0]  = MRT_Inf.Ud;
    oUdq[1]  = MRT_Inf.Uq;

    oSpd[0] = MRT_Inf.Spd;

    oSMO[0] = SMO.Ix;
    oSMO[1] = SMO.Iy;
    oSMO[2] = SMO.Ex;
    oSMO[3] = SMO.Ey;
    oSMO[4] = SMO.ThetaE;
    oSMO[5] = SMO.SpdE;
    oSMO[6] = SMO.Qx;
    oSMO[7] = SMO.Qy;
    oSMO[8] = SMO.Dir;
}

/* 用于存储全局变量和运行时参数，在确定端口的宽度和采样时间后调用 */
#define MDL_SET_WORK_WIDTHS
#if defined(MDL_SET_WORK_WIDTHS) && defined(MATLAB_MEX_FILE)
static void mdlSetWorkWidths(SimStruct *S)
{
     /* 设置运行时参数的数量 */
    if (!ssSetNumRunTimeParams(S, ParameterNum)) return;

    /* 注册参数 */
    ssRegDlgParamAsRunTimeParam(S, 0, 0,  "Np",     ssGetDataTypeId(S, "double"));
    ssRegDlgParamAsRunTimeParam(S, 1, 1,  "Udc",    ssGetDataTypeId(S, "double"));
    ssRegDlgParamAsRunTimeParam(S, 2, 2,  "Ls",     ssGetDataTypeId(S, "double"));
    ssRegDlgParamAsRunTimeParam(S, 3, 3,  "Rs",     ssGetDataTypeId(S, "double"));
    ssRegDlgParamAsRunTimeParam(S, 4, 4,  "Kt",     ssGetDataTypeId(S, "double"));
    ssRegDlgParamAsRunTimeParam(S, 5, 5,  "J",      ssGetDataTypeId(S, "double"));
    ssRegDlgParamAsRunTimeParam(S, 6, 6,  "Ld",     ssGetDataTypeId(S, "double"));
    ssRegDlgParamAsRunTimeParam(S, 7, 7,  "Lq",     ssGetDataTypeId(S, "double"));
}
#endif

/* 模块结束函数 */
static void mdlTerminate(SimStruct *S)
{
    
}

#ifdef MATLAB_MEX_FILE
/* 被Mex-file编译 */
#include "simulink.c"
#else
/* 不被Mex-file编译 */
#include "cg_sfun.h"
#endif
