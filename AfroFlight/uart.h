#pragma once

typedef struct _UARTVersion {
    u8 Hardware;
    u8 Software;
    u8 Protocol;
} _UARTVersion;

enum {
    UART_REQ_VERSION                    = 1 << 0,
    UART_REQ_ADCDATA                    = 1 << 1,
    UART_REQ_ADCDATACONT                = 1 << 2,
    UART_REQ_ADCDATASTOP                = 1 << 3,
    
    UART_REQ_REBOOT                     = 1 << 7
};

void UART_Init(void);
void UART_Transmit(u8 cmd, u8 nblocks, ...);
void UART_ReceiveTelemetry(void);
void UART_TransmitTelemetry(void);
