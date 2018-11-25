//
// stepper.c - stepper control library, for PCB laser
//
// v1.5 / 2018-06-19 / Io Engineering / Terje
//
// Calculations based 1200 DPI resolution with GT2 belt and 17 teeth pulleys: 1195.904 steps per inch (vs 1200) -> +0,085mm error/inch (scale factor: 1.0034)
// Driver must be set to 8 microsteps for a 200 steps/rev motor when steps per pixels is configured to 1
// Real scale factor for my CNC router is 1.0055
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
#include <stdint.h>
#include <stdbool.h>

#include "stepper.h"
#include "profile.h"

#define XHOMEFLAG 0x01
#define YHOMEFLAG 0x02
#define ZHOMEFLAG 0x04

static uint8_t homeFlags = 0;
static uint16_t rampCounter, fspCounter;
static struct axis xAxis, yAxis, zAxis, *yzAxis;

static void setDefaults (axis *axis, uint8_t stppx)
{
	axis->dir        = 2; // 2 means not set
	axis->position   = 0;
	axis->bComp      = 0;
	axis->cComp      = 0;
	axis->stppx      = stppx;
	axis->homeOffset = 0;
}

void stepperInit (void)
{

// X-axis

    XSTEP_CCR0 = accprofile[0];         // Set inital step time and
    XSTEP_CCR1 = STEPPULSE;             // pulse width
    XSTEP_CCTL1 = OUTMOD_7;             // Set output mode to PWM,
    XSTEP_CTL = TASSEL1|TACLR;          // bind to SMCLK and clear TA
#ifdef XSTEP_PORT_SEL
    XSTEP_PORT_SEL |= XSTEP_BIT;        // Enable TA0.1 on XSTEP pin
#endif
#ifdef XSTEP_PORT_SEL2
    XSTEP_PORT_SEL2 &= ~XSTEP_BIT;
#endif
    XSTEP_PORT_DIR |= XSTEP_BIT;        // Set X step and

#ifdef XDIR_PORT_SEL
    XDIR_PORT_SEL &= ~XDIR_BIT;         // Enable TA0.1 on XSTEP pin
#endif
#ifdef XDIR_PORT_SEL2
    XDIR_PORT_SEL2 &= ~XDIR_BIT;
#endif

    XDIR_PORT_DIR |= XDIR_BIT;          // dir pins as outputs

    XHOME_PORT_DIR &= ~XHOME_BIT;	    // Enable XHOME pin as input
    XHOME_PORT_OUT |= XHOME_BIT;        // and
    XHOME_PORT_REN |= XHOME_BIT;        // enable pullup


#ifdef XMOT_PORT
#if XMOT_PORT == 2 && XMOT_BIT == BIT7
	XMOT_PORT_SEL &= ~XMOT_BIT;         // Enable TA1.0 on Y/ZSTEP pin
	XMOT_PORT_SEL2 &= ~XMOT_BIT;        // Enable secondary function for XDIR and XSTEP
#endif
    XMOT_PORT_DIR |= XMOT_BIT;
#endif

	setDefaults(&xAxis, XSTPPIXELS);

	xAxis.homeOffset = 150;

// Y and Z-axis - NOTE: axes shares the same timer so cannot be driven simultaneously

    YSTEP_CCR0 = accprofile[0];         // Set initial step time and
    YSTEP_CCR1 = STEPPULSE;             // pulse width
    YSTEP_CCTL1 = OUTMOD_7;             // Set output mode to PWM,
    YSTEP_CTL = TASSEL1|TACLR;          // bind to SMCLK and clear TA
    YSTEP_PORT_SEL |= YSTEP_BIT;        // Enable TA1.0 on Y/ZSTEP pins
	YSTEP_PORT_DIR |= YSTEP_BIT;        // Set step pins as outputs

	YDIR_PORT_DIR |= YDIR_BIT;          // Set Y  & Z direction pins as output

	YHOME_PORT_DIR &= ~YHOME_BIT;       // Enable Y & ZHOME pins as input
    YHOME_PORT_OUT |= YHOME_BIT;        // and
    YHOME_PORT_REN |= YHOME_BIT;        // enable pullup
#ifdef YMOT_PORT
    YMOT_PORT_DIR |= YMOT_BIT;
#endif

    setDefaults(&yAxis, YSTPPIXELS);

#ifdef HAS_FOCUS
    ZSTEP_PORT_SEL |= ZSTEP_BIT;        // Enable TA1.0 on Y/ZSTEP pins
    ZSTEP_PORT_DIR |= ZSTEP_BIT;        // Set step pins as outputs

    ZDIR_PORT_DIR |= ZDIR_BIT;          // Set Y  & Z direction pins as output
    ZHOME_PORT_DIR &= ~ZHOME_BIT;       // Enable Y & ZHOME pins as input
    ZHOME_PORT_OUT |= ZHOME_BIT;        // and
    ZHOME_PORT_REN |= ZHOME_BIT;        // enable pullup
#ifdef ZMOT_PORT
    ZMOT_PORT_DIR |= ZMOT_BIT;
#endif

	setDefaults(&zAxis, YSTPPIXELS);
#endif

	ESTOP_PORT_DIR &= ~ESTOP_BIT;	    // Set ESTOP pin as input
	ESTOP_PORT_OUT |= ESTOP_BIT;        // and
	ESTOP_PORT_REN |= ESTOP_BIT;        // enable pullup

	enableMotors(false);
}

struct axis *calcParams (const int16_t pixels, struct axis *axis)
{
	uint16_t steps = abs(pixels);
	uint8_t dir = pixels >= 0 ? 1 : 0;

	axis->busy = State_Idle;
	axis->fsSteps = steps & 0x0001;
	steps = steps & 0xFFFE;
	axis->fsSteps += steps > RAMPSTEPS ? steps - RAMPSTEPS : 0;
	axis->rampSteps = (axis->fsSteps < 2 ? steps : RAMPSTEPS) >> 1;
	axis->profile = &accprofile[0];

	if (axis->dir != dir) {

		axis->cComp = axis->bComp;
		axis->dir = dir;

	    if(axis == &xAxis) {
	        if(dir)
	            XDIR_PORT_OUT |= XDIR_BIT;
	        else
	            XDIR_PORT_OUT &= ~XDIR_BIT;
	    } else if(axis == &yAxis) {
	        if(dir)
	            YDIR_PORT_OUT &= ~YDIR_BIT;
	        else
	            YDIR_PORT_OUT |= YDIR_BIT;
	    }
#ifdef HAS_FOCUS
	    else {
	        if(dir)
	            ZDIR_PORT_OUT &= ~ZDIR_BIT;
	        else
	            ZDIR_PORT_OUT |= ZDIR_BIT;
	    }
#endif
	    __delay_cycles(50000);
	}

	return axis;
}

struct axis *calcXParams (const int16_t pixels)
{
	return calcParams(pixels, &xAxis);
}

switches_t switch_status (void) {

    switches_t status = {0};

    status.xHome = (XHOME_PORT_IN & XHOME_BIT) != 0;
    status.yHome = (YHOME_PORT_IN & YHOME_BIT) != 0;
#ifdef HAS_FOCUS
    status.zHome = (ZHOME_PORT_IN & ZHOME_BIT) != 0;
#endif
    status.eStop = (ESTOP_PORT_IN & ESTOP_BIT) != 0;

    return status;
}

int16_t getXPos (void)
{
	return xAxis.position;
}

void setXHomePos (uint16_t homeOffset)
{
	xAxis.homeOffset = homeOffset;
}

void setXBacklashComp (uint16_t steps)
{
	xAxis.bComp = steps;
}

int16_t getYPos(void)
{
	return yAxis.position;
}

void setYHomePos (uint16_t homeOffset)
{
	yAxis.homeOffset = homeOffset;
}

int getZPos (void) {
	return zAxis.position;
}

void zeroAll (void)
{
	xAxis.position = 0;
	yAxis.position = 0;
	zAxis.position = 0;
}

void enableMotors (bool on)
{
#ifdef XMOT_PORT
    if(on) {
        XMOT_PORT_OUT &= ~XMOT_BIT;
#if (YMOT_PORT_OUT != XMOT_PORT_OUT) || (YMOT_BIT != XMOT_BIT)
        YMOT_PORT_OUT &= ~YMOT_BIT;
#endif
#if (ZMOT_PORT_OUT != YMOT_PORT_OUT) || (ZMOT_BIT != YMOT_BIT)
        ZMOT_PORT_OUT &= ~ZMOT_BIT;
#endif
        __delay_cycles(5000); // allow motors to settle
    } else {
        XMOT_PORT_OUT |= XMOT_BIT;
#if (YMOT_PORT_OUT != XMOT_PORT_OUT) || (YMOT_BIT != XMOT_BIT)
        YMOT_PORT_OUT |= YMOT_BIT;
#endif
#if (ZMOT_PORT_OUT != YMOT_PORT_OUT) || (ZMOT_BIT != YMOT_BIT)
        ZMOT_PORT_OUT |= ZMOT_BIT;
#endif
    }
#endif
}

void stopAll (void)
{
    XSTEP_CTL &= ~MC0;     		                        // Stop X step timer,
	YSTEP_CTL &= ~MC0;     		                        // YZ step timer and
	xAxis.busy = yAxis.busy = zAxis.busy = State_Idle;	// set all axes to idle
}

void toggleXDir (void)
{
    XDIR_PORT_OUT ^= XDIR_BIT;
	xAxis.cComp = xAxis.bComp;
	xAxis.dir = !xAxis.dir;
}

void homeAll (void)
{
	homeX();
	homeY();
#ifdef HAS_FOCUS
	homeZ();
#endif
}

static void homeXseq (const int16_t reverse, const int16_t forward)
{
	homeFlags |= XHOMEFLAG;

	moveX(reverse);

	homeFlags &= ~XHOMEFLAG;

	moveX(forward);
}

static void homeYseq (const int16_t reverse, const int16_t forward)
{
	homeFlags |= YHOMEFLAG;

	moveY(reverse);

	homeFlags &= ~YHOMEFLAG;

	moveY(forward);
}

void homeX (void)
{
    enableMotors(true);

	xAxis.bComp = 0;


	if(XHOME_PORT_IN & XHOME_BIT) 	// already home
		moveX(150);  	            // so move out a bit

	XSTEP_CTL &= ~(ID0|ID1);
	XSTEP_CTL |= ID0;
	homeXseq(-15000, 100);

	XSTEP_CTL |= ID1;
	homeXseq(-300, 50);
	homeXseq(-300, xAxis.homeOffset);

	XSTEP_CTL &= ~(ID0|ID1);
	xAxis.position = 0;
}

void homeY (void)
{
    enableMotors(true);

    yAxis.bComp = 0;

	YSTEP_CTL |= ID0;

	if(YHOME_PORT_IN & YHOME_BIT) 	// already home
		moveY(150);  	            // so move out a bit

	homeYseq(-15000, 100);

	YSTEP_CTL &= ~ID0;
	YSTEP_CTL |= ID1;
	homeYseq(-300, 50);
	homeYseq(-300, yAxis.homeOffset);

	YSTEP_CTL &= ~ID1;
	yAxis.position = 0;
}

void homeZ (void)
{
#ifdef HAS_FOCUS
    enableMotors(true);

    if(ZHOME_PORT_IN & ZHOME_BIT) 	// already home
		moveZ(100);  	            // so move out a bit

	homeFlags |= ZHOMEFLAG;

	moveZ(-30000);

	homeFlags &= ~ZHOMEFLAG;
#endif
	zAxis.position = 0;
}

void stopX (void)
{
	XSTEP_CTL &= ~MC0;          // Stop X step timer 0 and
	xAxis.busy = State_Idle;    // and set state IDLE
}

void startX (void)
{
    xAxis.cStppx = xAxis.stppx;
	xAxis.busy = xAxis.rampSteps ? State_Accel : (xAxis.fsSteps == 1 ? State_Decel : State_Move);

	rampCounter = xAxis.rampSteps;
	fspCounter = xAxis.fsSteps;
	XSTEP_CCR0 = *xAxis.profile;
	XSTEP_CCTL0 = CCIE;             // Enable X step timer interrupt
	XSTEP_CTL |= MC0;               // and start timer in up mode
}

bool moveX (const int16_t steps)
{
	if(steps == 0 || (ESTOP_PORT_IN & ESTOP_BIT))
		return false;

    calcParams(steps, &xAxis);

	startX();

	while(xAxis.busy) {
		LPM0;
		if((homeFlags & XHOMEFLAG) && (XHOME_PORT_IN & XHOME_BIT)) {
			XSTEP_CTL &= ~MC0;     // Stop X step timer 0
			xAxis.busy = State_Idle;
		}
	}

	return (ESTOP_PORT_IN & ESTOP_BIT) == 0;
}

static void startYZ (const int16_t steps, struct axis *axis)
{
	yzAxis = axis;

	calcParams(steps, yzAxis);

    yzAxis->cStppx = yzAxis->stppx;
	yzAxis->busy = yzAxis->rampSteps ? State_Accel : (yzAxis->fsSteps == 1 ? State_Decel : State_Move);

	rampCounter = yzAxis->rampSteps;
	fspCounter = yzAxis->fsSteps;

	YSTEP_CCR0 = *(yzAxis->profile);
	YSTEP_CCTL0 = CCIE;                           // Enable Y step timer interrupt
	YSTEP_CTL |= MC0|ID0;                         // and start timer in up mode
}

bool moveY (const int16_t steps)
{
	if(steps == 0 || (ESTOP_PORT_IN & ESTOP_BIT))
		return false;

#ifdef HAS_FOCUS
	YSTEP_PORT_SEL &= ~ZSTEP_BIT;
	YSTEP_PORT_SEL |= YSTEP_BIT;
#endif

	startYZ(steps, &yAxis);

	while(yzAxis->busy) {
		LPM0;
		if((homeFlags & YHOMEFLAG) && (YHOME_PORT_IN & YHOME_BIT)) {
			YSTEP_CTL &= ~MC0;     // Stop Y step timer 0
			yzAxis->busy = State_Idle;
		}
	}

	return (ESTOP_PORT_IN & ESTOP_BIT) == 0;
}

bool moveZ (const int16_t steps)
{
#ifdef HAS_FOCUS
	if(steps == 0 || (ESTOP_PORT_IN & ESTOP_BIT))
		return false;

#ifndef __MSP430F5310__
	ZSTEP_PORT_SEL  &= ~YSTEP_BIT;
	ZSTEP_PORT_SEL  |= ZSTEP_BIT;
#endif

	startYZ(steps, &zAxis);

	while(yzAxis->busy) {
		LPM0;
		if((homeFlags & ZHOMEFLAG) && (ZHOME_PORT_IN & ZHOME_BIT)) {
			yzAxis->busy = State_Idle;
			YSTEP_CTL &= ~MC0;     // Stop Y step timer 0
		}
	}

	return (ESTOP_PORT_IN & ESTOP_BIT) == 0;
#else
	return false;
#endif
}

// CCR0 Interrupt service routine - handles X-axis movement

#pragma vector=XSTEP_IRQH
//TIMER0_A0_VECTOR
__interrupt void XSTEP_ISR(void)
{
	if(ESTOP_PORT_IN & ESTOP_BIT)
		xAxis.busy = State_Idle;
#if	XSTPPIXELS > 1
	else if(--xAxis.cStppx == 0 && !xAxis.cComp) {
#else
	else if(!xAxis.cComp) {
#endif
		switch(xAxis.busy) {

			case State_Accel:
				if(--rampCounter) {
					XSTEP_CCR0 = *(++xAxis.profile); // Update timer timeout
				} else {
					xAxis.busy = fspCounter == 0 ? State_Decel : State_Move;
					rampCounter = xAxis.rampSteps;
				}
				break;

			case State_Move:
				if(!(--fspCounter))
					xAxis.busy = State_Decel;
				break;

			case State_Decel:
				if(rampCounter && --rampCounter)
					XSTEP_CCR0 = *(--xAxis.profile); // Update timer timeout
				else
					xAxis.busy = State_Idle;
				break;

		}

        if(xAxis.dir)
            xAxis.position++;
        else
            xAxis.position--;
	}

	if(!xAxis.busy)	{		// If task completed (or ESTOP)
		XSTEP_CTL &= ~MC0;  // stop X step timer
		xAxis.cComp = 0;	// and clear backlash counter
	}
#if XSTPPIXELS > 1
    if(!xAxis.cStppx) {
        xAxis.cStppx = xAxis.stppx;
        if(xAxis.cComp)
            xAxis.cComp--;
        else
            LPM0_EXIT;        	// Exit LPM0
    }
#else
    if(xAxis.cComp)
        xAxis.cComp--;
    else
        LPM0_EXIT;          // Exit LPM0
#endif
}

// CCR1 Interrupt service routine - handles Y & Z-axis movement

#pragma vector=YSTEP_IRQH
//TIMER1_A0_VECTOR
__interrupt void YZSTEP_ISR(void)
{
	if(ESTOP_PORT_IN & ESTOP_BIT)
		yzAxis->busy = State_Idle;
#if YSTPPIXELS > 1
    else if(--yzAxis->cStppx == 0) {
#else
    else {
#endif
	    switch(yzAxis->busy) {

			case State_Accel:
				if(--rampCounter) {
					YSTEP_CCR0 = *(++(yzAxis->profile)); // Update timer timeout
				} else {
					yzAxis->busy = fspCounter == 0 ? State_Decel : State_Move;
					rampCounter = yzAxis->rampSteps;
				}
				break;

			case State_Move:
				if(!(--fspCounter))
					yzAxis->busy = State_Decel;
				break;

			case State_Decel:
				if(rampCounter && --rampCounter)
					YSTEP_CCR0 = *(--(yzAxis->profile)); // Update timer timeout
				else
					yzAxis->busy = State_Idle;
				break;

		}

		if(yzAxis->dir)
			yzAxis->position++;
		else
			yzAxis->position--;
	}

	if(!yzAxis->busy)		// If task completed (or ESTOP)
		YSTEP_CTL &= ~MC0;  // stop Y step timer

#if YSTPPIXELS > 1
	 if(!yzAxis->cStppx) {
	    yzAxis->cStppx = yzAxis->stppx;
        LPM0_EXIT;				// Exit LPM0
	 }
#else
     LPM0_EXIT;              // Exit LPM0
#endif
}
