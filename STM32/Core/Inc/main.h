/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include "arm_math.h"
#include "filt.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum{
    START = 0,
    RUN,
    STOP
}ModeChange_Status_enum;

typedef enum{
    VolLoop = 0,
    CurLoop,
    SpdLoop,
    HFI_Sensorless,
    HFIwithSMO_Sensorless,
    SpdwithHFI
}ControlMode_enum;

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
    int8_t Spd_Tick;
    int8_t Start_Flag;
    ModeChange_Status_enum Status_Flag;
    int16_t Stop_cnt;
    
    float Spd;
    float Spd_rpm;
    float Spd_Target;
    float Spd_Target_rpm;
    float Uq_Target;
    float Id_Target;

    float Id;
    float Iq;

    float CurTs;
    float SpdTs;
    float CurFs;
    float SpdFs;

    float Ud;
    float Uq;

    ControlMode_enum Mode;
    ControlMode_enum Mode_last;
    
    float wc_Current;
    float wc_Speed;
}ControlCommand_str;

typedef struct{
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
    float Spd_rpm;

    float Udc;

    float SinTheta;
    float CosTheta; 

    float Ux;       
    float Uy;

    float U1;    
    float U2;   
    float U3;       

    uint8_t Sector;

    float CCRa;
    float CCRb; 
    float CCRc;

    float Ia;
    float Ib;
    float Ic;
    float Iin;

    float Ix;       
    float Iy;

    float Id;
    float Iq;
    float Iq_Ave;

    float Ud;
    float Uq;

    float EMF_Peak;
    float EMF_Rms;

    float Ud_qCoupling;
    float Uq_dCoupling;

    float Ud_Electrical;
    float Uq_Electrical;

    float Ud_ElectricalMaxUp;
    float Ud_ElectricalMaxDown;
    float Uq_ElectricalMaxUp;
    float Uq_ElectricalMaxDown;

    float Ex;
    float Ey;

    float ThetaE;
    
    float Tx;
    float Ty;
    
    float Ta;
    float Tb; 
    float Tc;
    
    float Uac;
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
    float Ix_Bef;
    float Iy_Bef;
    float Ex;
    float Ey;
    float Ex_Test;
    float Ey_Test;
    float Ix;
    float Iy;
    float Vx;
    float Vy;
    float h1;
    float h2;
    float de;
    PI_str SpdE_PI;
    float Spd;
    float Spd_rpm;
    float SpdE;
    float ThetaE;
    float SinTheta;
    float CosTheta;
    float Flag;
    float E1;
    float E2;
    float EMF_LPF_wc;
    float Exy_LPF_wc;
    float Theta_PLL_wn;
    float Theta_PLL_we;
    float Theta_PLL_zeta;
    float Spd_LPF_wc;
    float Switch_Spd;
    float Switch_EMF;
    float EMF_Flag;
    float EMF_Peak;
    float EMF_Rms;
    float EMF_Rms_Test;
    int8_t Dir;
    int8_t DirP;
    int8_t DirN;
    int8_t EMF_Dir;
    int8_t Dir_status;
    float Flux;
    uint8_t status;
    int8_t Qx;
    int8_t Qy;
}SlidingModeObserver_str;

typedef struct{
    uint32_t Theta;
    uint32_t ThetaE;
    uint32_t Theta_Pre;
    int16_t Ia;
    int16_t Ic;
    int16_t Ia_Ave;
    int16_t Ic_Ave;
    uint16_t Udc;
    int16_t Iin;
    uint8_t Udc_Ready;
    uint8_t Ia_Ready;
    uint8_t Ic_Ready;
    uint8_t Theta_Ready;
    uint8_t ADC1_DMA_Ready;
    uint8_t ADC2_DMA_Ready;
    uint8_t Encoder_Ready;
}SensorData_str;

typedef struct{
    float a1;
    float a2;
    float b1;
    float c1;
    float d1;
    float d2;
    float m;
}HFI_Rec_str;

typedef enum{
    HFI_WORK = 0,
    HFI_SMO_MIX,
    SMO_WORK
}HFIRun_Status_enum;

typedef enum{
    IH_INIT = 0,
    THETAE_INIT,
    POLE_INIT,
    POLE_N_WAIT,
    POLE_S_WAIT,
    HFI_WAIT
}HFIStart_Status_enum;

typedef struct{
    float Theta;
    float ThetaE;
    float delta_ThetaE;
    float SinTheta;
    float CosTheta;
    float SinTheta_Rec;
    float CosTheta_Rec;
    float Vh;
    float Vdh;
    float VPulse;
    int8_t Pulse_Max_cnt;

    float Ud;
    float Uq;

    float Ux;
    float Uy;
    float Ux_temp;
    float Uy_temp;

    float Ix;
    float Iy;
    float Ix_temp;
    float Iy_temp;
    float Ixh;
    float Iyh;
    float Ih;
    float PLL_wn;
    float PLL_zeta;
    float Spd_LPF_wc;
    int8_t Dir;
    PI_str SpdE_PI;
    float SpdE;
    float Spd;
    uint8_t Ready;
    float ThetaE_Err;
    float ThetaE_Err2;
    float Ih_Err;
    int32_t Start_cnt;
    float Ih_LPF_wc;
    float ThetaE_Err_LPF_wc;
    int32_t Pulse_cnt;
    float Ipulse_Max;
    float Ipulse_Min;
    HFI_Rec_str Rec;
    float ThetaE_Rec;
    float ThetaE_Rec2;
    float ThetaE_Rec_temp;
    HFIStart_Status_enum Start_Status;
    HFIRun_Status_enum Run_Status;
    HFIRun_Status_enum Run_Status_temp;
    int32_t status_cnt;
    float SpdE_Rec;
    float Spd_Rec;
    float SinThetaE_Rec;
    float CosThetaE_Rec;
}HighFrequencyInjection_str;

typedef union{
	uint8_t PC_uint8[4];
	float Pc_float;
}PCFloatData_union;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SDabc_Pin LL_GPIO_PIN_15
#define SDabc_GPIO_Port GPIOB
#define INa_Pin LL_GPIO_PIN_8
#define INa_GPIO_Port GPIOA
#define INb_Pin LL_GPIO_PIN_9
#define INb_GPIO_Port GPIOA
#define INc_Pin LL_GPIO_PIN_10
#define INc_GPIO_Port GPIOA
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/* USER CODE BEGIN Private defines */
#define TIM_Fre (uint32_t)(170000000)
#define PWM_Fre (uint32_t)(20000)
#define Timer_PERIOD (uint32_t)(TIM_Fre / PWM_Fre / 2)
#define PWM_phA_Init (uint32_t)(Timer_PERIOD / 2)
#define PWM_phB_Init (uint32_t)(Timer_PERIOD / 2)
#define PWM_phC_Init (uint32_t)(Timer_PERIOD / 2)
#define Vref 3.0f

#define TRUE 1
#define FALSE 0
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
