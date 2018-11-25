//
// serial.c - serial (UART) port library including MCP4725 DAC support, for PCB laser
//
// v1.5 / 2018-07-01 / Io Engineering / Terje
//
//

/*

Copyright (c) 2015-2018, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

· Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

· Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

· Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#include "serial.h"

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

char txbuf[TX_BUFFER_SIZE];
char rxbuf[RX_BUFFER_SIZE];

const char eol[] = "\r\n";
static volatile uint16_t tx_head = 0, tx_tail = 0, rx_head = 0, rx_tail = 0, rx_overflow = 0;

#ifdef XONXOFF
    static volatile unsigned int rx_off = XONOK;
#endif

inline static void setUARTBR (int prescaler)
{
    SERIAL_BR0 = prescaler & 0xFF;  // LSB
    SERIAL_BR1 = prescaler >> 8;    // MSB
}

void serialInit (void)
{
    SERIAL_SEL  |= RXD|TXD;         // P1.1 = RXD, P1.2=TXD
#ifdef SERIAL_SEL2
    SERIAL_SEL2 |= RXD|TXD;         // P1.1 = RXD, P1.2=TXD
#endif
    SERIAL_CTL1 = UCSWRST;
    SERIAL_CTL1 |= UCSSEL_2;        // Use SMCLK

#ifdef __MSP430F5310__
    setUARTBR(656);                 // Set baudrate to 38400 @ 25MHz SMCLK
    SERIAL_MCTL = 0;                // Modulation UCBRSx=0, UCBRFx=0
#else
    setUARTBR(26);                  // Set baudrate to 38400
    SERIAL_MCTL = UCBRF0|UCOS16;    // with oversampling
#endif

    SERIAL_CTL0 = 0;
    SERIAL_CTL1 &= ~UCSWRST;        // Initialize USCI state machine
    SERIAL_IE |= SERIAL_RXIE;       // Enable USCI_A0 RX interrupt

#ifndef XONXOFF
    RTS_PORT_DIR |= RTS_BIT;        // Enable RTS pin as output
    RTS_PORT_OUT &= ~RTS_BIT;       // and drive it low
#endif
}

uint16_t serialTxCount (void)
{
  uint16_t tail = tx_tail;
  return BUFCOUNT(tx_head, tail, TX_BUFFER_SIZE);
}

uint16_t serialRxCount (void)
{
  uint16_t tail = rx_tail, head = rx_head;
  return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

void serialRxFlush (void)
{
    rx_head = 0;
    rx_tail = 0;
    RTS_PORT_OUT &= ~RTS_BIT;
}

void serialPutC (const char c)
{
    uint16_t next_head = tx_head;

    if((SERIAL_IFG & SERIAL_TXIE) && next_head == tx_tail)  // If no data pending in buffers
        SERIAL_TXD = c;                                     // send character immediately

    else {

        if (++next_head == TX_BUFFER_SIZE)  // Wrap buffer index
            next_head = 0;                  // if at end

        while(tx_tail == next_head);        // Buffer full, block until free room

        txbuf[tx_head] = c;                 // Enter data into buffer
        tx_head = next_head;                // and increment pointer to next entry

        SERIAL_IE |= SERIAL_TXIE;           // Enable transmit interrupts
    }
}

char serialRead (void)
{
    uint16_t c, tail;

    tail = rx_tail;
    c = tail != rx_head ? rxbuf[tail++] : EOF;      // Get next character and increment pointer if data available else EOF
    rx_tail = tail == RX_BUFFER_SIZE ? 0 : tail;    // then update pointer

#ifdef XONXOFF
    if (rx_off == XOFFOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) {
        rx_off = XON;                               // Queue XON at front
        UC0IE |= SERIAL_TXIE;                       // and enable UART TX interrupt
    }
#else

    if ((RTS_PORT_IN & RTS_BIT) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM)  // Clear RTS if
        RTS_PORT_OUT &= ~RTS_BIT;                                                               // buffer count is below low water mark

#endif

    return c;
}

void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

void serialWriteLn (const char *data)
{
    serialWriteS(data);
    serialWriteS(eol);
}

void serialWrite (const char *data, uint16_t length)
{
    char *ptr = (char *)data;

    while(length--)
        serialPutC(*ptr++);
}

#ifdef HAS_DAC

/* MCP 4725 libray functions */

static uint8_t TXBuffer[3];         // Transmit buffer
static uint8_t *pTXBuffer;          // Pointer to TX buffer
static volatile uint8_t TXCount;    // Bytes to send

void setVoltage (uint16_t value, bool writeEEPROM)
{
    TXBuffer[0] = writeEEPROM ? MCP4725_WRITEEEPROM : MCP4725_WRITE;    // DAC command
    TXBuffer[1] = (value & 0xFF0) >> 4;                                 // MSB data
    TXBuffer[2] = (value & 0x00F) << 4;                                 // LSB data

    pTXBuffer = TXBuffer;               // TX buffer start address
    TXCount = 3;                        // Bytes to transmit

    while (I2C_CTL1 & UCTXSTP);         // Ensure stop condition got sent

    I2C_CTL1 |= UCTR|UCTXSTT;           // I2C TX, start condition

    LPM0;                               // Stay in LPM until finished
}

void resetDAC (void)
{
    TXBuffer[0] = 0x06;

    pTXBuffer = TXBuffer;               // TX buffer start address
    TXCount = 1;                        // Bytes to transmit

    while (I2C_CTL1 & UCTXSTP);         // Ensure stop condition got sent

    I2C_CTL1 |= UCTR|UCTXSTT;           // I2C TX, start condition

    LPM0;                               // Enter LPM0
}

void wakeUpDAC (void)
{
    TXBuffer[0] = 0x09;

    pTXBuffer = (uint8_t *)TXBuffer;    // TX buffer start address
    TXCount = 1;                        // Bytes to transmit

    while (I2C_CTL1 & UCTXSTP);         // Ensure stop condition got sent

    I2C_CTL1 |= UCTR|UCTXSTT;           // I2C TX, start condition

    LPM0;                               // Enter LPM0
}

void initDAC (void)
{
    I2C_SEL  |= SDC|SDA;                // Assign I2C pins to USCI_B0
#ifdef I2C_SEL2
    I2C_SEL2 |= SDC|SDA;                // Assign I2C pins to USCI_B0
#endif
    I2C_CTL1 |= UCSWRST;                // Enable SW reset
    I2C_CTL0 = UCMST|UCMODE_3|UCSYNC;   // I2C Master, synchronous mode
    I2C_CTL1 = UCSSEL_2|UCSWRST;        // Use SMCLK, keep SW reset
#ifdef __MSP430F5310__
    I2C_BR0 = 250;                      // fSCL = 100kHz
#else
    I2C_BR0 = 160;                      // fSCL = 100kHz
#endif
    I2C_BR1 = 0;
    I2C_CSA = 0x00;                     // Slave Address 0 for General Call
    I2C_CTL1 &= ~UCSWRST;               // Clear SW reset, resume operation
    I2C_IE |= SERIAL_TXIE;              // Enable TX interrupt

    resetDAC();                         // Reset
    wakeUpDAC();                        // and wakeup DAC

    while (I2C_CTL1 & UCTXSTP);         // Ensure stop condition got sent

    I2C_CSA = 0x60;                     // Slave Address is 048h
}

#endif

#ifdef __MSP430F5310__

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI1RX_ISR(void)
{
    uint16_t iv = UCA1IV;

    if(iv == 0x02) {

        uint16_t next_head = rx_head + 1;       // Get and increment buffer pointer

        if (next_head == RX_BUFFER_SIZE)        // If at end
            next_head = 0;                      // wrap pointer around

        if(rx_tail == next_head) {              // If buffer full
            rx_overflow = 1;                    // flag overlow
            next_head = SERIAL_RXD;             // and do dummy read to clear interrupt
        } else {
            rxbuf[rx_head] = SERIAL_RXD;        // Add data to buffer
            rx_head = next_head;                // and update pointer
    #ifdef XONXOFF
            if (rx_off == XONOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) > RX_BUFFER_HWM) {
                rx_off = XOFF;                  // Queue XOFF at front
                UC0IE |= UCA1TXIE;              // and enable UART TX interrupt
            }
    #else
            if (!(RTS_PORT_IN & RTS_BIT) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM) {

                RTS_PORT_OUT |= RTS_BIT;
                tx_head = tx_tail;
            }
    #endif
        }
    }

    if(iv == 0x04) {

        uint16_t tail = tx_tail;                // Get buffer pointer

    #ifdef XONXOFF
        if(rx_off == XON || rx_off == XOFF) {   // If we have XOFF/XON to send
            SERIAL_TXD = rx_off;                // send it
            rx_off |= 0x80;                     // and flag it sent
        } else {
    #endif
            SERIAL_TXD = txbuf[tail++];         // Send next character and increment pointer

            if(tail == TX_BUFFER_SIZE)          // If at end
                tail = 0;                       // wrap pointer around

            tx_tail = tail;                     // Update global pointer
    #ifdef XONXOFF
        }
    #endif
        if(tail == tx_head)                     // If buffer empty then
            SERIAL_IE &= ~SERIAL_TXIE;          // disable UART TX interrupt
    }

}

#pragma vector=USCI_B1_VECTOR
__interrupt void USCIB1_ISR(void)
{
    switch(__even_in_range(UCB1IV, 12)) {
        case  0: break;                         // Vector  0: No interrupts
        case  2: break;                         // Vector  2: ALIFG
        case  4:                                // Vector  4: NACKIFG
            if(I2C_STAT & UCNACKIFG) {
                I2C_STAT &= ~UCNACKIFG;
                I2C_CTL1 |= UCTXSTP;            // I2C stop condition
                LPM0_EXIT;                      // Exit LPM0
            }
            break;
        case  6: break;                         // Vector  6: STTIFG
        case  8: break;                         // Vector  8: STPIFG
        case 10: break;                         // Vector 10: RXIFG
        case 12: // Vector 12: TXIFG
            if(TXCount) {                       // Still data to send?
                I2C_TXD = *pTXBuffer++;         // Yep, load TX buffer and
                TXCount--;                      // decrement TX byte counter
            } else {
                I2C_CTL1 |= UCTXSTP;            // Send I2C stop condition and
                I2C_IFG &= ~SERIAL_TXIFG;       // clear TX int flag
                LPM0_EXIT;                      // Exit LPM0
            }
            break;
        default: break;
    }
}

#else

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    uint16_t next_head = rx_head + 1;       // Get and increment buffer pointer

    if (next_head == RX_BUFFER_SIZE)        // If at end
        next_head = 0;                      // wrap pointer around

    if(rx_tail == next_head) {              // If buffer full
        rx_overflow = 1;                    // flag overlow
        next_head = SERIAL_RXD;             // and do dummy read to clear interrupt
    } else {
        rxbuf[rx_head] = SERIAL_RXD;        // Add data to buffer
        rx_head = next_head;                // and update pointer
#ifdef XONXOFF
        if (rx_off == XONOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) > RX_BUFFER_HWM) {
            rx_off = XOFF;                  // Queue XOFF at front
            SERIAL_IE |= SERIAL_TXIE;       // and enable UART TX interrupt
        }
#else
        if (!(RTS_PORT_IN & RTS_BIT) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
            RTS_PORT_OUT |= RTS_BIT;
#endif
    }
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    if((SERIAL_IFG & SERIAL_TXIE) && (SERIAL_IE & SERIAL_TXIE)) { // UART

        uint16_t tail = tx_tail;                // Get buffer pointer

    #ifdef XONXOFF
        if(rx_off == XON || rx_off == XOFF) {   // If we have XOFF/XON to send
            SERIAL_TXD = rx_off;                // send it
            rx_off |= 0x80;                     // and flag it sent
        } else {
    #endif
            SERIAL_TXD = txbuf[tail++];         // Send next character and increment pointer

            if(tail == TX_BUFFER_SIZE)          // If at end
                tail = 0;                       // wrap pointer around

            tx_tail = tail;                     // Update global pointer
    #ifdef XONXOFF
        }
    #endif
        if(tail == tx_head)                     // If buffer empty then
            SERIAL_IE &= ~SERIAL_TXIE;          // disable UART TX interrupt
    }

#ifdef HAS_DAC
    if((SERIAL_IFG & UCB0TXIFG) && (SERIAL_IE & UCB0TXIFG)) { // I2C

        if(I2C_STAT & UCNACKIFG) {
            I2C_STAT &= ~UCNACKIFG;
            I2C_CTL1 |= UCTXSTP;                // I2C stop condition
            LPM0_EXIT;                          // Exit LPM0
        }

        if(TXCount) {                           // Still data to send?
            I2C_TXD = *pTXBuffer++;             // Yep, load TX buffer
            TXCount--;                          // and decrement TX byte counter
        } else {
            I2C_CTL1 |= UCTXSTP;                // Send I2C stop condition and
            I2C_IFG &= ~UCB0TXIFG;              // clear TX int flag
            LPM0_EXIT;                          // Exit LPM0
        }
    }
#endif

}

#endif
