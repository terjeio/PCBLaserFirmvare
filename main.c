//
// main.c - PCB exposer (for 405nm laser diode) - MSP430G2553
//
// v1.2 / 2017-02-08 / Io Engineering / Terje
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
#include "serial.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "stepper.h"
#include "serial.h"

#define LASER BIT5 // P2.5

#define PXSTEPS 1
#define BYTEPIXELS 7
#define BYTEOFFSET 59

#define MSG_ABOUT 0U
#define MSG_ESTOP 1U
#define MSG_PARAM 2U
#define MSG_RUN   3U
#define MSG_OK    4U
#define MSG_FAIL  5U
#define MSG_BADC  6U

char const *const message[] = {
	"\r\nIo Engineering PCB Laser rev 1.2\0",
	"ESTOP!",
	"Error: missing parameters",
	"Job running...",
	"OK",
	"FAILED",
	"Bad command"
};

char const *const command[] = {
	"?",
	"XPix:",
	"YPix:",
	"Start",
	"MoveX:",
	"MoveY:",
	"MoveZ:",
	"Laser:",
	"Power:",
	"ZeroAll",
	"HomeXY",
	"HomeX",
	"HomeY",
	"HomeZ",
	"Echo:",
	"XHome:"
	"YHome:"
	"XBComp:"
};

#define NUMCOMMANDS 18

const char cr = '\r';
const char lf = '\n';

const char ysteps[] = {6,5,6,5,6,6,6,5,6,5,6,6,6,5,6,5,6,6,6,5,6,5,6,5,6,5,6,5,6,6,6,5,6,5,6,6,6,5,6,5,6,6,6,5,6};
const char ystepn = sizeof(ysteps) / sizeof(ysteps[0]);

char cmdbuf[16];
unsigned int pxCounter, cancelled = 0, echo = 1, bcomp = 9;

bool waitForRowStart (uint16_t line) {

	uint16_t chr, count = 0, wait;

	pxCounter = BYTEPIXELS;

	while(serialRxCount() < 100); // wait for RX buffer to fill up a bit before proceeding

	if((chr = serialRead()) != '\n')
		cancelled = 1;

	wait = !cancelled;

	while(wait) {
		if(serialRxCount()) {
			if((chr = serialRead()) == ':') {
				wait = 0;
				cmdbuf[count] = '\0';
				// validate linenumber?
			} else if(chr > 31 && count < 15)
				cmdbuf[count++] = chr;
			else if(chr == EOF) {
				wait = 0;
				cancelled = 1;
			}
		}
	}

	if(!cancelled && (chr = serialRead()) == EOF)
		cancelled = 1;

	return chr - BYTEOFFSET;

}

bool renderRow (unsigned int x, unsigned int y) {

	struct axis *axis;
	uint16_t pixels, yseq = 0, rows = y;

	if(isESTOP())
		return false;

	serialWriteLn(message[MSG_RUN]);

	cancelled = 0;

	axis = calcXParams(x - 1);

	setXBacklashComp(bcomp);

	zeroAll();

	while(!cancelled && rows--) {

		pixels = waitForRowStart(y - rows);

		if(!cancelled)
			startX();

		while(axis->busy) {

			if(pixels & 0x01)
				P2OUT |= LASER;
			else
				P2OUT &= ~LASER;

			if(--pxCounter)
				pixels = pixels >> 1;

			else {

				pxCounter = BYTEPIXELS;

				if((pixels = serialRead()) == EOF) {
					stopX();
					cancelled = 1;
				}

				pixels -= BYTEOFFSET;

			}

			if(axis->busy)
				LPM0;

		}

		if(!cancelled) {

			P2OUT |= LASER; 		// Switch off laser

			moveY(ysteps[yseq++]);	// Move table one pixel in Y direction
			toggleXDir();			// and change X direction

			if(yseq == ystepn)
				yseq = 0;

		}

	}

	if(cancelled)
		serialRxFlush();

	stopAll();

	moveX(-getXPos());			// Move table back
	moveY(-getYPos());			// to origin

	P2OUT |= LASER; 			// switch off laser (ESTOP may have been requested)

	return !isESTOP() && !cancelled;

}

int parseInt (char *s) {

	int c, res = 0, negative = 0;

	if(*s == '-') {
		negative = 1;
		s++;
	} else if(*s == '+')
		s++;

	while((c = *s++) != '\0') {
		if(c >= 48 && c <= 57)
			res = (res * 10) + c - 48;
	}

	return negative ? -res : res;

}

unsigned char parseCommand (char *cmd) {

	unsigned char x = 0, hit = 0;

	while(!hit && x < NUMCOMMANDS) {

		if(!strncmp(command[x], cmd, strlen(command[x])))
			hit = 1;
		else
			x++;

	}

	return x;

}

void main(void) {

	char cmd;
	unsigned int cmdptr = 0, xpixels = 0, ypixels = 0;

	WDTCTL = WDTPW | WDTHOLD;				// Stop watchdog timer
	DCOCTL = CALDCO_16MHZ;                  // Set DCO for 16MHz using
	BCSCTL1 = CALBC1_16MHZ;                 // calibration registers

	P1DIR = 0xFF;                           // All P1 outputs
	P1OUT = 0;                              // Clear P1 outputs
	P2DIR |= LASER;                         // Enable laser output on P2
//	P3DIR = 0xFF;                           // All P3 outputs
//	P3OUT = 0;                              // Clear P3 outputs

	P2OUT |= LASER; 						// switch off laser!

	serialInit();
	stepperInit();

	_EINT();                                // Enable interrupts

	initDAC();

	serialRxFlush();
	serialWriteLn(message[MSG_ABOUT]);
	serialWriteLn(message[isESTOP() ? MSG_ESTOP : MSG_OK]);

	while(1) {

		if(serialRxCount()) { // bytes waiting, process them

			cmd = serialRead();

			if(echo)
				serialPutC(cmd);

			if(cmd == cr && cmdptr > 0) {

				cmdbuf[cmdptr] = 0;

				if(echo)
					serialPutC(lf);

				switch(parseCommand(cmdbuf)) {

					case 0: 	// ?
						serialWriteLn(message[MSG_ABOUT]);
						serialWriteLn(message[isESTOP() ? MSG_ESTOP : MSG_OK]);
						break;

					case 1: 	// XPix
						xpixels = parseInt(&(cmdbuf[5]));
						break;

					case 2:		// YPix
						ypixels = parseInt(&(cmdbuf[5]));
						break;

					case 3:		// Start
						if(xpixels == 0 || ypixels == 0)
							serialWriteLn(message[MSG_PARAM]);
						else
							serialWriteLn(message[renderRow(xpixels, ypixels) ? MSG_OK : MSG_FAIL]);
						break;

					case 4:  	// MoveX
						serialWriteLn(message[moveX(parseInt(&(cmdbuf[6]))) ? MSG_OK : MSG_FAIL]);
						break;

					case 5:  	// MoveY
						serialWriteLn(message[moveY(parseInt(&(cmdbuf[6]))) ? MSG_OK : MSG_FAIL]);
						break;

					case 6:  	// MoveZ
						serialWriteLn(message[moveZ(parseInt(&(cmdbuf[6]))) ? MSG_OK : MSG_FAIL]);
						break;

					case 7: 	// Laser (0|1)
						if(parseInt(&(cmdbuf[6])) == 0)
							P2OUT |= LASER;						// switch off laser
						else
							P2OUT &= ~LASER; 					// switch on laser
						serialWriteLn(message[MSG_OK]);
						break;

					case 8: 	// Power
						serialWriteLn(message[MSG_OK]);
						setVoltage(parseInt(&(cmdbuf[6])), 0);
						break;

					case 9:		// ZeroAll
						zeroAll();
						serialWriteLn(message[MSG_OK]);
						break;

					case 10:	// HomeXY
						homeX();
						homeY();
						serialWriteLn(message[MSG_OK]);
						break;

					case 11:	// HomeX
						homeX();
						serialWriteLn(message[MSG_OK]);
						break;

					case 12:	// HomeY
						homeY();
						serialWriteLn(message[MSG_OK]);
						break;

					case 13:	// HomeZ
						homeZ();
						serialWriteLn(message[MSG_OK]);
						break;

					case 14:	// Echo
						echo = parseInt(&(cmdbuf[5])) != 0;
						break;

					case 15:  	// XHome
						setXHomePos(parseInt(&(cmdbuf[6])));
						serialWriteLn(message[MSG_OK]);
						break;

					case 16:  	// YHome
						setYHomePos(parseInt(&(cmdbuf[6])));
						serialWriteLn(message[MSG_OK]);
						break;

					case 17:  	// XBComp
						bcomp = parseInt(&(cmdbuf[6]));
						serialWriteLn(message[MSG_OK]);
						break;

					default:	// Bad command
						serialWriteLn(message[MSG_BADC]);
						break;

				}

				cmdptr = 0;
				cmdbuf[0] = 0;

			} else if(cmd != lf) {
				cmdbuf[cmdptr++] = cmd;
				cmdptr &= 0x0F;
			}
		}
	}
}
