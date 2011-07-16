#pragma once

typedef struct _UARTVersion {
    u8 Hardware;
    u8 Software;
    u8 Protocol;
} _UARTVersion;

enum {
    UART_REQ_VERSION                    = 1 << 0,
    UART_REQ_ADCDATA                    = 1 << 1
};

void UART_Init(void);
void UART_Transmit(u8 cmd, u8 nblocks, ...);
void UART_ReceiveTelemetry(void);
void UART_TransmitTelemetry(void);
