//
// serial.c - serial (UART) port library including MCP4725 DAC support, for PCB laser
//
// v1.0 / 2016-06-03 / Io Engineering / Terje
//
//

/*

Copyright (c) 2015, Terje Io
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

#include "serial.h"

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

char txbuf[TX_BUFFER_SIZE];
char rxbuf[RX_BUFFER_SIZE];

const char eol[] = "\r\n";
static volatile unsigned int tx_head = 0, tx_tail = 0, rx_head = 0, rx_tail = 0, rx_overflow = 0;

#ifdef XONXOFF
	static volatile unsigned int rx_off = XONOK;
#endif

void setUCA0BR (int prescaler) {
	UCA0BR0 = prescaler & 0xFF; // LSB
	UCA0BR1 = prescaler >> 8;	// MSB
}

void serialInit(void) {

	P1SEL  |= RXD|TXD; 	// P1.1 = RXD, P1.2=TXD
	P1SEL2 |= RXD|TXD; 	// P1.1 = RXD, P1.2=TXD

	UCA0CTL1 = UCSWRST;
	UCA0CTL1 |= UCSSEL_2; 	// Use SMCLK

/*	setUCA0BR(138); 			// Set baud rate to 115200 with 1MHz clock (Data Sheet 15.3.13)
	UCA0MCTL = UCBRS0+UCBRS1+UCBRS2; 		// Modulation UCBRSx = 1
*/
	setUCA0BR(26);				// Set baudrate to 38400
	UCA0MCTL = UCBRF0|UCOS16; 	// with oversampling

	UCA0CTL0 = 0;
	UCA0CTL1 &= ~UCSWRST; 	// Initialize USCI state machine
	IE2 |= UCA0RXIE; 		// Enable USCI_A0 RX interrupt

#ifndef XONXOFF
	P1DIR |= RTS;			// Enable RTS pin
	P1OUT &= ~RTS;			// and drive it low
#endif

}

unsigned int serialTxCount(void)
{
  unsigned int tail = tx_tail;
  return BUFCOUNT(tx_head, tail, TX_BUFFER_SIZE);
}

unsigned int serialRxCount(void)
{
  unsigned int tail = rx_tail, head = rx_head;
  return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

void serialRxFlush(void)
{
	rx_head = 0;
	rx_tail = 0;
	P1OUT &= ~RTS;
}

void serialPutC(const char data) {

	unsigned int next_head = tx_head + 1;

	if (next_head == TX_BUFFER_SIZE)
		next_head = 0;

	while(tx_tail == next_head); 	// Buffer full, block until free room

 	txbuf[tx_head] = data;			// Enter data into buffer
	tx_head = next_head;			// and increment pointer to next entry

	UC0IE |= UCA0TXIE;				// Enable transmit interrupts

}

char serialRead(void) {

	unsigned int data, tail;

	_DINT();                                				// Disable interrupts (to avoid contention)

	tail = rx_tail;
	data = tail != rx_head ? rxbuf[tail++] : EOF; 			// Get next character and increment pointer if data available else EOF
	rx_tail = tail == RX_BUFFER_SIZE ? 0 : tail;   			// then update pointer

#ifdef XONXOFF
    if (rx_off == XOFFOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) {
    	rx_off = XON;										// Queue XON at front
    	UC0IE |= UCA0TXIE; 									// and enable UART TX interrupt
    }
#else

    if ((P1IN & RTS) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM)	// Clear RTS if
    	P1OUT &= ~RTS;										// buffer count is below low water mark

    _EINT();                                				// Reenable interrupts

#endif

    return data;
}

void serialWriteS(const char *data) {

	char c, *ptr = (char *)data;

	while((c = *ptr++) != '\0')
		serialPutC(c);

}

void serialWriteLn(const char *data) {
	serialWriteS(data);
	serialWriteS(eol);
}

void serialWrite(const char *data, unsigned int length) {

	char *ptr = (char *)data;

	while(length--)
		serialPutC(*ptr++);

}

/* MCP 4725 libray functions */

#define SDC BIT6
#define SDA BIT7

unsigned static char TXBuffer[3];			// Transmit buffer
unsigned static char *pTXBuffer;			// Pointer to TX buffer
unsigned static char TXCount;				// Bytes to send

void setVoltage (unsigned int value, bool writeEEPROM) {

	unsigned int stat = UCB0CTL1 & UCTXSTP;

	TXBuffer[0] = writeEEPROM ? MCP4725_WRITEEEPROM : MCP4725_WRITE; 	// DAC command
	TXBuffer[1] = (value & 0xFF0) >> 4;						    	   	// MSB data
	TXBuffer[2] = (value & 0x00F) << 4;									// LSB data

	pTXBuffer = (unsigned char *)TXBuffer;  // TX buffer start address
	TXCount = 3;              				// Bytes to transmit

	while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition

    while(TXCount)							// While data to send
    	LPM0;        						// enter LPM until finished

}

void resetDAC (void) {

	TXBuffer[0] = 0x06;

	pTXBuffer = (unsigned char *)TXBuffer;  // TX buffer start address
	TXCount = 1;              				// Bytes to transmit

	while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition

    LPM0;        							// Enter LPM0

}

void wakeUpDAC (void) {

	TXBuffer[0] = 0x09;

	pTXBuffer = (unsigned char *)TXBuffer;  // TX buffer start address
	TXCount = 1;              				// Bytes to transmit

	while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    UCB0CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition

    LPM0;        							// Enter LPM0

}

void initDAC (void) {

	P1SEL  |= SDC|SDA;                     	// Assign I2C pins to USCI_B0
	P1SEL2 |= SDC|SDA;                     	// Assign I2C pins to USCI_B0
	UCB0CTL1 |= UCSWRST;                    // Enable SW reset
	UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;   // I2C Master, synchronous mode
	UCB0CTL1 = UCSSEL_2 + UCSWRST;          // Use SMCLK, keep SW reset
	UCB0BR0 = 160;                          // fSCL = SMCLK/3 = ~333kHz
	UCB0BR1 = 0;
	UCB0I2CSA = 0x00;                       // Slave Address 0 for General Call
	UCB0CTL1 &= ~UCSWRST;                   // Clear SW reset, resume operation
	IE2 |= UCB0TXIE;                        // Enable TX interrupt

	resetDAC();								// Reset
	wakeUpDAC();							// and wakeup DAC

	while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

	UCB0I2CSA = 0x60;                       // Slave Address is 048h
}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{

	unsigned int next_head = rx_head + 1;	// Get and increment buffer pointer

	if (next_head == RX_BUFFER_SIZE)		// If at end
		next_head = 0;						// wrap pointer around

	if(rx_tail == next_head) {				// If buffer full
		rx_overflow = 1;					// flag overlow
		next_head = UCA0RXBUF; 				// and do dummy read to clear interrupt
	} else {
		rxbuf[rx_head] = UCA0RXBUF;			// Add data to buffer
		rx_head = next_head;				// and update pointer
#ifdef XONXOFF
		if (rx_off == XONOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) > RX_BUFFER_HWM) {
	    	rx_off = XOFF;					// Queue XOFF at front
	    	UC0IE |= UCA0TXIE; 				// and enable UART TX interrupt
	    }
#else
		if (!(P1IN & RTS) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
			P1OUT |= RTS;
#endif
	}

}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{

	if(IFG2 & UCA0TXIFG) { // UART

		unsigned int tail = tx_tail;			// Get buffer pointer

	#ifdef XONXOFF
		if(rx_off == XON || rx_off == XOFF) {	// If we have XOFF/XON to send
			UCA0TXBUF = rx_off; 			  	// send it
			rx_off |= 0x80;						// and flag it sent
		} else {
	#endif
			UCA0TXBUF = txbuf[tail++]; 			// Send next character and increment pointer

			if(tail == TX_BUFFER_SIZE) 			// If at end
				tail = 0;						// wrap pointer around

			tx_tail = tail;						// Update global pointer
	#ifdef XONXOFF
		}
	#endif
		if(tail == tx_head)						// If buffer empty then
		   UC0IE &= ~UCA0TXIE; 					// disable UART TX interrupt

	} else { // I2C

		if(UCB0STAT & UCNACKIFG) {
			UCB0STAT &= ~UCNACKIFG;
			UCB0CTL1 |= UCTXSTP;				// I2C stop condition
			LPM0_EXIT;							// Exit LPM0
		}

		if(TXCount)								// Still data to send?
		{
			UCB0TXBUF = *pTXBuffer++;			// Load TX buffer
			TXCount--;							// Decrement TX byte counter
		} else {
			UCB0CTL1 |= UCTXSTP;				// I2C stop condition
			IFG2 &= ~UCB0TXIFG;           		// Clear USCI_B0 TX int flag
			LPM0_EXIT;							// Exit LPM0
		}

	}

}
