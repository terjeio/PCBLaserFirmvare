//
// stepper.c - stepper control library, for PCB laser
//
// v1.2 / 2017-02-08 / Io Engineering / Terje
//
// Calculations based on GT2 belt and 17 teeth pulleys: 1195.904 steps per inch (vs 1200) -> +0,085mm error/inch (scale factor: 1.0034)
// Driver is set to 8 microsteps for a 200 steps/rev motor
// Real scale factor for my CNC router is 1.0055
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
#include <stdint.h>
#include <stdbool.h>

#include "stepper.h"
#include "profile.h"

#define XHOMEFLAG 0x01
#define YHOMEFLAG 0x02
#define ZHOMEFLAG 0x04

static unsigned char homeFlags = 0;
static unsigned int rampCounter, fspCounter;
static struct axis xAxis, yAxis, zAxis, *yzAxis;

static void setDefaults (axis *axis, unsigned char dirBit) {

	axis->dir        = 1;
	axis->dirBit     = dirBit;
	axis->position   = 0;
	axis->bComp      = 0;
	axis->cComp      = 0;
	axis->homeOffset = 0;

}

void stepperInit (void) {

// X-axis

	P1DIR &= ~XHOME;						// Enable XHOME pin as input
	P1OUT |= XHOME;                        	// and
	P1REN |= XHOME;                        	// enable pullup

	TA0CCR0 = accprofile[0];                // Set inital step time and
	TA0CCR1 = STEPPULSE;                    // pulse width
	TA0CCTL1 = OUTMOD_7;                    // Set output mode to PWM,
	TA0CTL = TASSEL1+TACLR;                 // bind to SMCLK and clear TA

	setDefaults(&xAxis, XDIR);

	xAxis.homeOffset = 150;

// Y and Z-axis - NOTE: axes shares P2 TA1 so cannot be driven simultaneously

	P2SEL |= XSTEP|YSTEP;      				// Enable TA1.0 on Y/ZSTEP pin
	P2SEL &= ~(XDIR);      					// Enable TA1.0 on Y/ZSTEP pin
	P2SEL2 &= ~(XDIR|XSTEP);      			// Enable secondary function for XDIR and XSTEP

	P2DIR |= XDIR|XSTEP|YSTEP|ZSTEP;
	P3DIR |= YDIR|ZDIR;
	P2DIR &= ~(YHOME|ZHOME);				// Enable Y & ZHOME pins as input
	P2OUT |= YHOME|ZHOME;                   // and
	P2REN |= YHOME|ZHOME;                   // enable pullup

	TA1CCR0 = accprofile[0];                // Set initial step time and
	TA1CCR1 = STEPPULSE;                    // pulse width
	TA1CCTL1 = OUTMOD_7;                    // Set output mode to PWM,
	TA1CTL = TASSEL1+TACLR;                 // bind to SMCLK and clear TA

	setDefaults(&yAxis, YDIR);
	setDefaults(&zAxis, ZDIR);

	P3DIR &= ~ESTOP;						// Set ESTOP pin as input
	P3OUT |= ESTOP;                        	// and
	P3REN |= ESTOP;                        	// enable pullup

}

struct axis *calcParams (const int16_t pixels, struct axis *axis) {

	int steps = abs(pixels);
	int dir = pixels >= 0 ? 1 : 0;

	axis->busy = STATE_IDLE;
	axis->fsSteps = steps & 0x0001;
	steps = steps & 0xFFFE;
	axis->fsSteps += steps > RAMPSTEPS ? steps - RAMPSTEPS : 0;
	axis->rampSteps = (axis->fsSteps < 2 ? steps : RAMPSTEPS) >> 1;
	axis->profile = &accprofile[0];

	if (axis->dir != dir) {
		axis->cComp = axis->bComp;
		axis->dir = dir;
		__delay_cycles(50000);
	}

	return axis;

}

struct axis *calcXParams (const int16_t pixels) {
	P2OUT |= XDIR;
	return calcParams (pixels, &xAxis);
}

bool isESTOP (void) {
	return (P3IN & ESTOP) != 0;
}

int16_t getXPos(void) {
	return xAxis.position;
}

void setXHomePos (uint16_t homeOffset) {
	xAxis.homeOffset = homeOffset;
}

void setXBacklashComp (uint16_t steps) {
	xAxis.bComp = steps;
}

int16_t getYPos(void) {
	return yAxis.position;
}

void setYHomePos (uint16_t homeOffset) {
	yAxis.homeOffset = homeOffset;
}

int getZPos(void) {
	return zAxis.position;
}

void zeroAll (void) {
	xAxis.position = 0;
	yAxis.position = 0;
	zAxis.position = 0;
}

void stopAll (void) {
	TA0CTL &= ~MC0;     		// stop timer 0,
	TA1CTL &= ~MC0;     		// timer 1 and
	xAxis.busy = STATE_IDLE;	// set all
	yAxis.busy = STATE_IDLE;	// axes to
	zAxis.busy = STATE_IDLE;	// idle
}

void toggleXDir (void) {
	P2OUT ^= XDIR;		// toggle X direction
	xAxis.cComp = xAxis.bComp;
}

void homeAll (void) {
	homeX();
	homeY();
	homeZ();
}

static void homeXseq (const int16_t reverse, const int16_t forward) {

	homeFlags |= XHOMEFLAG;

	moveX(reverse);

	homeFlags &= ~XHOMEFLAG;

	moveX(forward);

}

static void homeYseq (const int16_t reverse, const int16_t forward) {

	homeFlags |= YHOMEFLAG;

	moveY(reverse);

	homeFlags &= ~YHOMEFLAG;

	moveY(forward);

}

void homeX (void) {

	xAxis.bComp = 0;

	if(P1IN & XHOME) 	// already home
		moveX(500);  	// so move out a bit

	TA0CTL &= ~(ID0|ID1);
	TA0CTL |= ID0;
	homeXseq(-15000, 100);

	TA0CTL |= ID1;
	homeXseq(-300, 50);
	homeXseq(-300, xAxis.homeOffset);

	TA0CTL &= ~(ID0|ID1);
	xAxis.position = 0;

}

void homeY (void) {

	yAxis.bComp = 0;

	TA1CTL |= ID0;

	if(P2IN & YHOME) 	// already home
		moveY(500);  	// so move out a bit

	homeYseq(-15000, 100);

	TA1CTL &= ~ID0;
	TA1CTL |= ID1;
	homeYseq(-300, 50);
	homeYseq(-300, yAxis.homeOffset);

	TA1CTL &= ~ID1;
	yAxis.position = 0;

}

void homeZ (void) {

	if(P2IN & ZHOME) 	// already home
		moveZ(100);  	// so move out a bit

	homeFlags |= ZHOMEFLAG;

	moveZ(-30000);

	homeFlags &= ~ZHOMEFLAG;

	zAxis.position = 0;

}

void stopX (void) {
	TA0CTL &= ~MC0;     // stop timer 0 and
	xAxis.busy = STATE_IDLE;
}

void startX (void) {

	xAxis.busy = xAxis.rampSteps ? STATE_ACCEL : STATE_MOVE;

	rampCounter = xAxis.rampSteps;
	fspCounter = xAxis.fsSteps;
	TA0CCR0 = *xAxis.profile;
	TA0CCTL0 = CCIE;                           // Enable CCR0 interrupt|
	TA0CTL |= MC0;                             // Start TA0 in up mode
}

bool moveX (const int16_t steps) {

	if(steps == 0 || P3IN & ESTOP)
		return false;

	if(steps > 0)
		P2OUT |= XDIR;
	else
		P2OUT &= ~XDIR;

	calcParams(steps, &xAxis);

	startX();

	while(xAxis.busy) {
		LPM0;
		if((homeFlags & XHOMEFLAG) && (P1IN & XHOME)) {
			TA0CTL &= ~MC0;     // stop timer 0 and
			xAxis.busy = 0;
		}
	}

	return (P3IN & ESTOP) == 0;

}

void startYZ (const int16_t steps, struct axis *axis) {

	yzAxis = axis;

	calcParams(steps, yzAxis);

	if(steps > 0)
		P3OUT &= ~(axis->dirBit);
	else
		P3OUT |= axis->dirBit;

	yzAxis->busy = yzAxis->rampSteps ? STATE_ACCEL : STATE_MOVE;

	rampCounter = yzAxis->rampSteps;
	fspCounter = yzAxis->fsSteps;

	TA1CCR0 = *(yzAxis->profile);
	TA1CCTL0 = CCIE;                           // Enable CCR0 interrupt|
	TA1CTL |= MC0|ID0;                         // Start TA0 in up mode
}

bool moveY (const int16_t steps) {

	if(steps == 0 || P3IN & ESTOP)
		return false;

	P2SEL &= ~ZSTEP;
	P2SEL |= YSTEP;

	startYZ(steps, &yAxis);

	while(yzAxis->busy) {
		LPM0;
		if((homeFlags & YHOMEFLAG) && (P2IN & YHOME)) {
			TA1CTL &= ~MC0;     // stop timer 0 and
			yzAxis->busy = 0;
		}
	}

	return (P3IN & ESTOP) == 0;

}

bool moveZ (const int16_t steps) {

	if(steps == 0 || P3IN & ESTOP)
		return false;

	P2SEL &= ~YSTEP;
	P2SEL |= ZSTEP;

	startYZ(steps, &zAxis);

	while(yzAxis->busy) {
		LPM0;
		if((homeFlags & ZHOMEFLAG) && (P2IN & ZHOME)) {
			yzAxis->busy = 0;
			TA1CTL &= ~MC0;     // stop timer 0 and
		}
	}

	return (P3IN & ESTOP) == 0;

}

// CCR0 Interrupt service routine - handles X-axis movement

#pragma vector=TIMER0_A0_VECTOR
__interrupt void CCR0_ISR(void)
{

	if(P3IN & ESTOP)
		xAxis.busy = STATE_IDLE;

	else if(!xAxis.cComp) {

		switch(xAxis.busy) {

			case STATE_ACCEL:
				if(--rampCounter) {
					TA0CCR0 = *(++xAxis.profile);
				} else {
					xAxis.busy = fspCounter == 0 ? STATE_DECEL : STATE_MOVE;
					rampCounter = xAxis.rampSteps;
				}
				break;

			case STATE_MOVE:
				if(!(--fspCounter))
					xAxis.busy = STATE_DECEL;
				break;

			case STATE_DECEL:
				if(rampCounter && --rampCounter)
					TA0CCR0 = *(--xAxis.profile);
				else
					xAxis.busy = STATE_IDLE;
				break;

		}

		if(P2OUT & XDIR)
			xAxis.position++;
		else
			xAxis.position--;

	}

	if(!xAxis.busy)	{		// If task completed (or ESTOP)
		TA0CTL &= ~MC0;		// stop timer 0
		xAxis.cComp = 0;	// and clear backlash counter
	}

	if(xAxis.cComp)
		xAxis.cComp--;
	else
		LPM0_EXIT;        	// Exit LPM0

}

// CCR1 Interrupt service routine - handles Y & Z-axis movement

#pragma vector=TIMER1_A0_VECTOR
__interrupt void CCR1_ISR(void)
{

	if(P3IN & ESTOP) {
		yzAxis->busy = STATE_IDLE;

	} else {

		switch(yzAxis->busy) {

			case STATE_ACCEL:
				if(--rampCounter) {
					TA1CCR0 = *(++(yzAxis->profile));
				} else {
					yzAxis->busy = fspCounter == 0 ? STATE_DECEL : STATE_MOVE;
					rampCounter = yzAxis->rampSteps;
				}
				break;

			case STATE_MOVE:
				if(!(--fspCounter))
					yzAxis->busy = STATE_DECEL;
				break;

			case STATE_DECEL:
				if(rampCounter && --rampCounter)
					TA1CCR0 = *(--(yzAxis->profile));
				else
					yzAxis->busy = STATE_IDLE;
				break;

		}

		if(P3OUT & yzAxis->dirBit)
			yzAxis->position--;
		else
			yzAxis->position++;

	}

	if(!yzAxis->busy)		// If task completed (or ESTOP)
		TA1CTL &= ~MC0;     // stop timer 1 and

	LPM0_EXIT;				// Exit LPM0

}
