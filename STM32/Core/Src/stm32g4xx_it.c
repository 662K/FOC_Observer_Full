/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FOCSub.h"
#include "DataProcessing.h"
#include "filt.h"
#include "VOFA.h"
#include "FOC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENCODER_BUFFER_NUM 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int8_t UART_Cnt = 0;
uint8_t Encoder_Cnt = 0;
uint8_t Encoder_CRC = 0;
uint8_t Encoder_buffer[ENCODER_BUFFER_NUM] = {0};
uint8_t PC_Command = 0;
PCFloatData_union PC_buffer = {0};
FIFO_int16 Ic_FIFO = {.length = 500};
FIFO_int16 Ia_FIFO = {.length = 500};
uint16_t FIFO_Cnt = 0;
uint16_t MotorStatus = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(void);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */
extern MotorRealTimeInformation_str MRT_Inf;
extern MotorParameter_str MotorParameter;
extern SensorData_str SensorData;
extern uint16_t ADC1_Buffer[3];
extern uint16_t ADC2_Buffer;
extern PI_str D_PI;
extern PI_str Q_PI;
extern PI_str Spd_PI;
extern ControlCommand_str CtrlCom;
extern uint8_t UART2_Buffer[6];
extern SlidingModeObserver_str SMO;
extern Frame_union DataUpToPc;
extern HighFrequencyInjection_str HFI;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
    if(LL_DMA_IsActiveFlag_TC1(DMA1)){
        SensorData.ADC1_DMA_Ready = 1;
        
    }
    LL_DMA_ClearFlag_TC1(DMA1);
  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
    
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
    if(LL_DMA_IsActiveFlag_TC2(DMA1)){
        SensorData.ADC2_DMA_Ready = 1;
        SensorData.Udc_Ready = 1;
    }
    LL_DMA_ClearFlag_TC2(DMA1);
    
  /* USER CODE END DMA1_Channel2_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
    
  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */
    if(LL_DMA_IsActiveFlag_TC3(DMA1)){
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
//        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
    }
    LL_DMA_ClearFlag_TC3(DMA1);
  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */
    TIM1->SR &= ~TIM_SR_UIF;
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */
    Encoder_Cnt = 0;
    Encoder_CRC = 0;
    USART2->TDR = 0x02;
    
    switch(MotorStatus){
    case 0:
        if(SensorData.Theta_Ready == 1){
            MotorStatus = 1;
        }
        break;
    case 1:
        if(SensorData.Udc_Ready == 1){
            MotorStatus = 2;
            FIFO_Cnt = 0;
        }
        break;
    case 2:
        if(SensorData.Ic_Ready == 1){
            MotorStatus = 3;
            FIFO_Cnt = 0;
        }
        break;
    case 3:
        if(SensorData.Ia_Ready == 1) {
            MotorStatus = 4;
            FIFO_Cnt = 0;
            SensorData.Theta_Pre = SensorData.Theta;
        }
        break;
    }
    
    switch(MotorStatus){
        case 2:    
            if(SensorData.ADC1_DMA_Ready == 1){
                SensorData.ADC1_DMA_Ready = 0;
                
                if(FIFO_Cnt < 1000){
                    FIFO_DataUpdate_int16(&Ic_FIFO, ADC1_Buffer[0]);
                    FIFO_Cnt++;
                }
                else{
                    int16_t Ic_Ave = FIFO_Get_Ave_int16(&Ic_FIFO);
                    
                    if((Ic_Ave < 2098) && (Ic_Ave > 1998)){
                        SensorData.Ic_Ave = Ic_Ave;
                        SensorData.Ic_Ready = 1;
                    }
                    else{
                        FIFO_Cnt = 0;
                        SensorData.Ic_Ready = 0;
                    }
                }   
            }
            break;
        case 3:    
            if(SensorData.ADC1_DMA_Ready == 1){
                SensorData.ADC1_DMA_Ready = 0;
                
                if(FIFO_Cnt < 1000){
                    FIFO_DataUpdate_int16(&Ia_FIFO, ADC1_Buffer[1]);
                    FIFO_Cnt++;
                }
                else{
                    int16_t Ia_Ave = FIFO_Get_Ave_int16(&Ia_FIFO);
                    
                    if((Ia_Ave < 2098) && (Ia_Ave > 1998)){
                        SensorData.Ia_Ave = Ia_Ave;
                        SensorData.Ia_Ready = 1;
                    }
                    else{
                        FIFO_Cnt = 0;
                        SensorData.Ia_Ready = 0;
                    }
                }   
            }
            break;
        case 4:
            if(SensorData.ADC1_DMA_Ready == 1){
                SensorData.ADC1_DMA_Ready = 0;
                SensorData.Ic = ADC1_Buffer[0] - SensorData.Ic_Ave;
                SensorData.Ia = SensorData.Ia_Ave - ADC1_Buffer[1];
                SensorData.Iin = 2048 - ADC1_Buffer[2];
                
                MRT_Inf.Ia = (Vref / 2 * SensorData.Ia) / (1 << 11) / 50 / 0.006f;
                MRT_Inf.Ic = (Vref / 2 * SensorData.Ic) / (1 << 11) / 50 / 0.006f;
                MRT_Inf.Iin = (Vref / 2 * SensorData.Iin) / (1 << 11) / 50 / 0.003f;
            }
            if(SensorData.ADC2_DMA_Ready == 1){
                SensorData.ADC2_DMA_Ready = 0;
                SensorData.Udc = ADC2_Buffer;
                
                MRT_Inf.Udc = (Vref * SensorData.Udc)  / (1 << 12) * (470 + 15) / 15;
                MRT_Inf.Uac = MRT_Inf.Udc * 0.57735f;
            }
            
            MRT_Inf.Ib = -MRT_Inf.Ia - MRT_Inf.Ic;
            D_PI.Max = MRT_Inf.Uac;
            Q_PI.Max = MRT_Inf.Uac;
            
            if(SensorData.Encoder_Ready == 1){
                //SensorData.Encoder_Ready = 0;
                MRT_Inf.Theta = (PI * 2 * SensorData.Theta) / (1 << 17);
                MRT_Inf.ThetaE = (PI * 2 * SensorData.ThetaE) / (1 << 17);
            }
            
            if(UART_Cnt < 10){
                UART_Cnt++;
            }else{
                MRT_Inf.Spd_rpm = MRT_Inf.Spd * 9.5493f;
                CtrlCom.Spd_rpm = CtrlCom.Spd * 9.5493f;
                SMO.Spd_rpm = SMO.Spd * 9.5493f;

                DataUpToPc.FrameData.fdata[0] = MRT_Inf.Iq;
                DataUpToPc.FrameData.fdata[1] = MRT_Inf.Spd_rpm;
                DataUpToPc.FrameData.fdata[2] = MRT_Inf.ThetaE;
                DataUpToPc.FrameData.fdata[3] = HFI.ThetaE;
                DataUpToPc.FrameData.fdata[4] = HFI.ThetaE_Rec;
                DataUpToPc.FrameData.fdata[5] = HFI.ThetaE_Rec2;
                
                
//                LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
                LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
                
                UART_Cnt = 0;
            }
            CtrlComFilter(&CtrlCom.Spd, CtrlCom.Spd_Target, 0.03f);
            Spd_Timer(&(CtrlCom.Spd_Tick));
            GetSpd(SensorData.Theta, &(SensorData.Theta_Pre), &MRT_Inf.Spd, CtrlCom.SpdFs, CtrlCom.Spd_Tick, &CtrlCom.Start_Flag);

            FOC_Mode_Select(&D_PI, &Q_PI, &Spd_PI, &CtrlCom, &MotorParameter, &MRT_Inf, &SMO, &HFI);
            
            TIM1->CCR1 = (uint32_t)(MRT_Inf.CCRa * Timer_PERIOD);
            TIM1->CCR2 = (uint32_t)(MRT_Inf.CCRb * Timer_PERIOD);
            TIM1->CCR3 = (uint32_t)(MRT_Inf.CCRc * Timer_PERIOD);
            break;
    }
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    static uint8_t PC_Cnt = 0;
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
    if (LL_USART_IsActiveFlag_RXNE(USART1)){
        uint8_t UART1_Data = USART1->RDR;
        
        switch(PC_Cnt){
        case 0:
            if(UART1_Data == 0x00){
                PC_Cnt = 1;
            }
            else{
                PC_Cnt = 0;
            }
            break;
        case 1:
            if(UART1_Data == 0x00){
                PC_Cnt = 2;
            }
            else{
                PC_Cnt = 0;
            }
            break;
        case 2:
            if(UART1_Data == 0x80){
                PC_Cnt = 3;
            }
            else{
                PC_Cnt = 0;
            }
            break;
        case 3:
            if(UART1_Data == 0x7F){
                PC_Cnt = 4;
            }
            else{
                PC_Cnt = 0;
            }
            break;
        case 4:
            if(UART1_Data != 0x01){
                PC_Command = UART1_Data;
                PC_Cnt = 5;
            }
            else{
                PC_Cnt = 10;
            }
            break;
        case 5:
            PC_buffer.PC_uint8[0] = UART1_Data;
            PC_Cnt = 6;
            break;
        case 6:
            PC_buffer.PC_uint8[1] = UART1_Data;
            PC_Cnt = 7;
            break;
        case 7:
            PC_buffer.PC_uint8[2] = UART1_Data;
            PC_Cnt = 8;
            break;
        case 8:
            PC_buffer.PC_uint8[3] = UART1_Data;
            PC_Cnt = 9;
            break;
        case 9:
            if(UART1_Data == 0xAA){
                switch(PC_Command){
                case 0x02:
                    if(PC_buffer.Pc_float > 5){
                        CtrlCom.Uq_Target = 5;
                    }
                    else if(PC_buffer.Pc_float < -5){
                        CtrlCom.Uq_Target = -5;
                    }
                    else{
                        CtrlCom.Uq_Target = PC_buffer.Pc_float;
                    }
                    break;
                case 0x03:
                    if(PC_buffer.Pc_float > 1){
                        CtrlCom.Id_Target = 1;
                    }
                    else if(PC_buffer.Pc_float < -1){
                        CtrlCom.Id_Target = -1;
                    }
                    else{
                        CtrlCom.Id_Target = PC_buffer.Pc_float;
                    }
                    break;
                case 0x04:
                    if(PC_buffer.Pc_float > 3000){
                        CtrlCom.Spd_Target_rpm = 3000;
                    }
                    else if(PC_buffer.Pc_float < -3000){
                        CtrlCom.Spd_Target_rpm = -3000;
                    }
                    else{
                        CtrlCom.Spd_Target_rpm = PC_buffer.Pc_float;
                    }
                    CtrlCom.Spd_Target = CtrlCom.Spd_Target_rpm / 60 * 2 * PI;
                    break;
                }
            }
            PC_Command = 0;
            PC_Cnt = 0;
            break;
        case 10:
            PC_Command = UART1_Data;
            PC_Cnt = 11;
            break;
        case 11:
            if(UART1_Data == 0xAA){
                switch (PC_Command){
                case 0x00:
                    CtrlCom.Mode = VolLoop;
                    CtrlCom.Status_Flag = STOP;
                    break;
                case 0x01:
                    CtrlCom.Mode = CurLoop;
                    CtrlCom.Status_Flag = STOP;
                    break;
                case 0x02:
                    CtrlCom.Mode = SpdLoop;
                    CtrlCom.Status_Flag = STOP;
                    break;
                case 0x03:
                    CtrlCom.Mode = HFI_Sensorless;
                    CtrlCom.Status_Flag = STOP;
                    break;
                case 0x04:
                    CtrlCom.Mode = HFIwithSMO_Sensorless;
                    CtrlCom.Status_Flag = STOP;
                    break;
                case 0x05:
                    CtrlCom.Mode = SpdwithHFI;
                    CtrlCom.Status_Flag = STOP;
                    break;
                }
            }
            PC_Command = 0;
            PC_Cnt = 0;
            break;
        }
    }
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    
    if (LL_USART_IsActiveFlag_RXNE(USART2)){
        Encoder_buffer[Encoder_Cnt] = USART2->RDR;
        
        if(Encoder_Cnt != 5){
            Encoder_CRC ^= Encoder_buffer[Encoder_Cnt];
        }
        else{
            if((Encoder_CRC == Encoder_buffer[5]) && (Encoder_buffer[0] == 0x02)){
                SensorData.Theta = (~((Encoder_buffer[2] << 0) | (Encoder_buffer[3] << 8) | (Encoder_buffer[4] << 16))) & 0x1FFFF;
                SensorData.ThetaE = (SensorData.Theta * MotorParameter.Np) & 0x1FFFF;
                SensorData.Encoder_Ready = 1;
                SensorData.Theta_Ready = 1;
            }
        }
        
        Encoder_Cnt++;
    }
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */
    
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
