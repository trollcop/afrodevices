#include "main.h"
#include <stdarg.h>

#define TX_BUFFER_SIZE   (0x80)
#define RX_BUFFER_SIZE   (0x80)
#define countof(a)   (sizeof(a) / sizeof(*(a)))

static u8 TxBuffer[TX_BUFFER_SIZE];                                     // Transmit buffer
static u8 RxBuffer[RX_BUFFER_SIZE];                                     // Receive buffer
static u8 RxLocked = FALSE;                                             // After receiving full buffer lock it while main loop process it
static u8 TxComplete = FALSE;                                           // Transmission complete
static u8 TxBytes = 0;                                                  // When sending raw data, this is used to specify buffer length
static u8 RxBytes = 0;                                                  // number of bytes received
static u16 UartRequest = 0;                                             // What data do we want returned from FC?

static const _UARTVersion UARTVersion = {
    0x01,       // Hardware
    0x01,       // Software
    0x01,       // Protocol
};

__near __interrupt void UART2_TX_IRQHandler(void)
{
    static u8 buffer_ptr = 0;

    if (!TxComplete) {
        u8 ch;
        buffer_ptr++;
        ch = TxBuffer[buffer_ptr];      // 1st byte was already sent in UART_Transmit() to fire off the interrupt
        if (TxBytes == 0 || buffer_ptr == TX_BUFFER_SIZE)) {
            buffer_ptr = 0;
            TxComplete = TRUE;
        } else {
            // transmit it, this will call back interrupt
            UART2_SendData8(ch);
            TxBytes--;
        }
    } else {
        buffer_ptr = 0;
        UART2_ITConfig(UART2_IT_TXE, DISABLE);
        memset(TxBuffer, 0, sizeof(TxBuffer));
    }
}

__near __interrupt void UART2_RX_IRQHandler(void)
{
    static u8 buffer_ptr = 0;
    u8 ch;

    // receive byte
    ch = (u8)(UART2_ReceiveData8() & 0x7F);

    // if we haven't processed our buffer yet ignore it for now
    if (RxLocked)
        return;

    if (buffer_ptr < RX_BUFFER_SIZE) {
        if (ch == '\n') {
            // end of receive, lock buffer and fire off to processing
            RxBytes = buffer_ptr;
            buffer_ptr = 0; // reset it
            RxLocked = TRUE;
        } else {
            // fill buffer
            RxBuffer[buffer_ptr++] = ch;
        }
    } else {
        // overflow
        buffer_ptr = 0;
        RxLocked = FALSE;
    }
}

static const char hexcode[] = "0123456789ABCDEF";

static void UART_Decode(void)
{
    u8 len = RxBytes - 2;
    u8 *ptr = RxBuffer + 2;
    u8 *dptr = RxBuffer + 2;
    u8 ch1, ch2 = 0;
    
    while (len) {
        ch1 = *ptr++;
        ch2 = *ptr++;
        *dptr = (u8)((ch1 < 57) ? (ch1 - 48) : (ch1 - 55)) << 4 | (u8)((ch2 < 57) ? (ch2 - 48) : (ch2 - 55));
        dptr++;
        len -= 2;
    }
}

// Transmit blocks of data. each block is buffer * + size of buffer. can be more than one.
void UART_Transmit(u8 cmd, u8 nblocks, ...)
{
    va_list va;
    u8 pt = 0;
    u8 *data = NULL;
    u8 ptr = 0;
    u8 len = 0;
    u8 ch, b, c;

    va_start(va, nblocks);

    // sync char + cmd
    TxBuffer[pt++] = '$';
    TxBuffer[pt++] = cmd;
    
    if (nblocks) {
        data = va_arg(va, u8 *);
        len = (u8)va_arg(va, int);
        ptr = 0;
        nblocks--;
    }

    while (len) {
        if (len) {
            ch = data[ptr++];
            len--;
            if ((!len) && nblocks) {
                // TxBuffer[pt++] = '$';
                data = va_arg(va, u8 *);
                len = (u8)va_arg(va, int);
                ptr = 0;
                nblocks--;
            }
            TxBuffer[pt++] = hexcode[ch >> 4 & 0xF];
            TxBuffer[pt++] = hexcode[ch & 0xF];
        }
    }

    va_end(va);
    // last char
    TxBuffer[pt] = '\n';
    // number of bytes to send
    TxBytes = pt;
    // send off 1st byte to start the process
    TxComplete = FALSE;
    UART2_SendData8(TxBuffer[0]);
    UART2_ITConfig(UART2_IT_TXE, ENABLE);
}

void UART_TransmitRaw(u8 cmd, u8 nblocks, ...)
{
    va_list va;
    u8 pt = 0;
    u8 *data = NULL;
    u8 ptr = 0;
    u8 len = 0;
    u8 ch, b, c;

    va_start(va, nblocks);

    // sync char (0xAA twice)
    TxBuffer[pt++] = 0xAA;
    TxBuffer[pt++] = 0xAA;

    if (nblocks) {
        data = va_arg(va, u8 *);
        len = (u8)va_arg(va, int);
        ptr = 0;
        nblocks--;
    }

    while (len) {
        if (len) {
            ch = data[ptr++];
            len--;
            if ((!len) && nblocks) {
                TxBuffer[pt++] = ch;
                TxBuffer[pt++] = 0xAA;
                data = va_arg(va, u8 *);
                len = (u8)va_arg(va, int);
                ptr = 0;
                nblocks--;
            } else {
                TxBuffer[pt++] = ch;
            }
        }
    }

    va_end(va);
    // last char
    TxBuffer[pt] = '\n';
    // number of bytes to send
    TxBytes = pt;
    // send off 1st byte to start the process
    TxComplete = FALSE;
    UART2_SendData8(TxBuffer[0]);
    UART2_ITConfig(UART2_IT_TXE, ENABLE);
}

char putchar(char c)
{
    /* Write a character to the UART1 */
    UART2_SendData8(c);
    /* Loop until the end of transmission */
    while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET);
    return c;
}

void UART_Init(void)
{
    const char *welcome = "AfroFlightST rev 0";
    UART2_DeInit();
    UART2_Init((u32)57600, UART2_WORDLENGTH_8D, UART2_STOPBITS_1, UART2_PARITY_NO, UART2_SYNCMODE_CLOCK_DISABLE, UART2_MODE_TXRX_ENABLE);
    UART2_ITConfig(UART2_IT_TXE, ENABLE);
    UART2_ITConfig(UART2_IT_RXNE_OR, ENABLE);

    UART_Transmit('W', 1, welcome, strlen(welcome));
}

void UART_ReceiveTelemetry(void)
{
    if (!RxLocked)
        return;

    UART_Decode();

    switch (RxBuffer[1]) {
        case 'V':       // Version info
            FLAG_SET(UartRequest, UART_REQ_VERSION);
            break;
        case 'a':       // Continous analog values RAW
            FLAG_SET(UartRequest, UART_REQ_ADCDATACONT);
            break;
        case 'A':       // Analog values
            FLAG_SET(UartRequest, UART_REQ_ADCDATA);
            break;
        case 'R':       // Reboot
            FLAG_SET(UartRequest, UART_REQ_REBOOT);
            break;
        case 's':       // Stop continous analog values
            FLAG_CLEAR(UartRequest, UART_REQ_ADCDATACONT);
            break;
    }

    RxLocked = FALSE;
    RxBytes = 0;    
}

void UART_TransmitTelemetry(void)
{
    // send stuff out
    if (!TxComplete)
        return;

    if (FLAG_ISSET(UartRequest, UART_REQ_VERSION)) {
        UART_Transmit('V', 1, (u8 *)&UARTVersion, sizeof(UARTVersion));
        FLAG_CLEAR(UartRequest, UART_REQ_VERSION);
    }
    if (FLAG_ISSET(UartRequest, UART_REQ_ADCDATA)) {
        UART_Transmit('A', 3, (u8 *)gyro, sizeof(gyro), (u8 *)&battery, sizeof(s16), (u8 *)acc, sizeof(acc));
        FLAG_CLEAR(UartRequest, UART_REQ_ADCDATA);
    }
    if (FLAG_ISSET(UartRequest, UART_REQ_ADCDATACONT)) {
        UART_TransmitRaw('a', 3, (u8 *)gyro, sizeof(gyro), (u8 *)&battery, sizeof(s16), (u8 *)acc, sizeof(acc));
        // Don't clear this flag as its continous
    }
    if (FLAG_ISSET(UartRequest, UART_REQ_REBOOT)) {
        // reboot to bootloader
        WWDG_SWReset();
    }
}
