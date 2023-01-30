#include "VOFA.h"

void UART_Send(uint8_t* Data, uint32_t length){
    for(uint32_t i = 0; i < length; i++){
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        USART1->TDR = Data[i];
    }
}

void SendJustFloat(Frame_union* DataUpToPc){
    UART_Send(DataUpToPc->UartSendData, sizeof(DataUpToPc->FrameData));
}


