//
// serial.h - serial (UART) port library including MCP4725 DAC support, for PCB laser
//
// v1.0 / 2015-01-26 / Io Engineering / Terje
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

#include <stdbool.h>

#define XON  0x11
#define XOFF 0x13
#define EOF  0x1A

#define XONOK (XON|0x80)
#define XOFFOK (XOFF|0x80)
#define TX_BUFFER_SIZE 16
#define RX_BUFFER_SIZE 192
#define RX_BUFFER_HWM 188
#define RX_BUFFER_LWM 150

#define RXD BIT1 // P1.1
#define TXD BIT2 // P1.2
#define CTS BIT4 // P1.4
#define RTS BIT3 // P3.1

/* UART */

void serialInit (void);
unsigned int serialTxCount(void);
unsigned int serialRxCount(void);
void serialRxFlush (void);
char serialRead (void);
void serialPutC (const char data);
void serialWriteS (const char *data);
void serialWriteLn (const char *data);
void serialWrite (const char *data, unsigned int length);

/* MCP4725 DAC */

#define MCP4725_WRITE       (0x40)  // Writes data to the DAC
#define MCP4725_WRITEEEPROM (0x60)  // Writes data to the DAC and EEPROM

void setVoltage (unsigned int value, bool writeEEPROM);
void resetDAC (void);
void wakeUpDAC (void);
void initDAC (void) ;
