//
// stepper.h - stepper control library, for PCB laser
//
// v1.5 / 2018-06-19 / Io Engineering / Terje
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

#include "config.h"

#define STEPPULSE 50
#define RAMPSTEPS 200 // includes both leading and trailing ramp!

#define XSTEP_CTL timerCtl(XSTEP_TIMER, XSTEP_TIMERI)
#define XSTEP_CCR0 timerCcr(XSTEP_TIMER, XSTEP_TIMERI, 0)
#define XSTEP_CCR1 timerCcr(XSTEP_TIMER, XSTEP_TIMERI, 1)
#define XSTEP_CCTL0 timerCCtl(XSTEP_TIMER, XSTEP_TIMERI, 0)
#define XSTEP_CCTL1 timerCCtl(XSTEP_TIMER, XSTEP_TIMERI, 1)
#define XSTEP_IRQH timerInt(XSTEP_TIMER, XSTEP_TIMERI, 0)

#define YSTEP_CTL timerCtl(YSTEP_TIMER, YSTEP_TIMERI)
#define YSTEP_CCR0 timerCcr(YSTEP_TIMER, YSTEP_TIMERI, 0)
#define YSTEP_CCR1 timerCcr(YSTEP_TIMER, YSTEP_TIMERI, 1)
#define YSTEP_CCTL0 timerCCtl(YSTEP_TIMER, YSTEP_TIMERI, 0)
#define YSTEP_CCTL1 timerCCtl(YSTEP_TIMER, YSTEP_TIMERI, 1)
#define YSTEP_IRQH timerInt(YSTEP_TIMER, YSTEP_TIMERI, 0)

#define XSTEP_PORT_DIR  portDir(XSTEP_PORT)
#define XSTEP_PORT_SEL  portSel(XSTEP_PORT)
#if XSTEP_PORT == 2 && (XSTEP_BIT == BIT6 || XSTEP_BIT == BIT7)
#define XSTEP_PORT_SEL2 portSel2(XSTEP_PORT)
#endif

#define XDIR_PORT_IN    portIn(XDIR_PORT)
#define XDIR_PORT_OUT   portOut(XDIR_PORT)
#define XDIR_PORT_DIR   portDir(XDIR_PORT)
#if XDIR_PORT == 2 && (XDIR_BIT == BIT6 || XDIR_BIT == BIT7)
#define XDIR_PORT_SEL   portSel(XSTEP_PORT)
#define XDIR_PORT_SEL2  portSel2(XSTEP_PORT)
#endif

#ifdef XMOT_PORT
#define XMOT_PORT_OUT   portOut(XMOT_PORT)
#define XMOT_PORT_DIR   portDir(XMOT_PORT)
#define XMOT_PORT_SEL   portSel(XMOT_PORT)
#ifndef __MSP430F5310__
#define XMOT_PORT_SEL2  portSel2(XMOT_PORT)
#endif
#endif

#define XHOME_PORT_IN   portIn(XHOME_PORT)
#define XHOME_PORT_OUT  portOut(XHOME_PORT)
#define XHOME_PORT_DIR  portDir(XHOME_PORT)
#define XHOME_PORT_REN  portRen(XHOME_PORT)

#define YSTEP_PORT_DIR portDir(YSTEP_PORT)
#define YSTEP_PORT_SEL portSel(YSTEP_PORT)

#define YDIR_PORT_IN   portIn(YDIR_PORT)
#define YDIR_PORT_OUT  portOut(YDIR_PORT)
#define YDIR_PORT_DIR  portDir(YDIR_PORT)

#ifdef YMOT_PORT
#define YMOT_PORT_OUT  portOut(YMOT_PORT)
#define YMOT_PORT_DIR  portDir(YMOT_PORT)
#endif

#define YHOME_PORT_IN  portIn(YHOME_PORT)
#define YHOME_PORT_OUT portOut(YHOME_PORT)
#define YHOME_PORT_DIR portDir(YHOME_PORT)
#define YHOME_PORT_REN portRen(YHOME_PORT)

#ifdef HAS_FOCUS
#define ZSTEP_PORT_DIR portDir(ZSTEP_PORT)
#define ZSTEP_PORT_SEL portSel(ZSTEP_PORT)

#define ZDIR_PORT_IN   portIn(ZDIR_PORT)
#define ZDIR_PORT_OUT  portOut(ZDIR_PORT)
#define ZDIR_PORT_DIR  portDir(ZDIR_PORT)

#ifdef ZMOT_PORT
#define ZMOT_PORT_OUT  portOut(ZMOT_PORT)
#define ZMOT_PORT_DIR  portDir(ZMOT_PORT)
#endif

#define ZHOME_PORT_IN  portIn(ZHOME_PORT)
#define ZHOME_PORT_OUT portOut(ZHOME_PORT)
#define ZHOME_PORT_DIR portDir(ZHOME_PORT)
#define ZHOME_PORT_REN portRen(ZHOME_PORT)
#endif

#define ESTOP_PORT_DIR  portDir(ESTOP_PORT)
#define ESTOP_PORT_IN   portIn(ESTOP_PORT)
#define ESTOP_PORT_OUT  portOut(ESTOP_PORT)
#define ESTOP_PORT_REN  portRen(ESTOP_PORT)

typedef enum {
    State_Idle = 0,
    State_Accel,
    State_Move,
    State_Decel
} stepper_state_t;

typedef struct axis {
    uint8_t bComp;
    uint8_t cComp;
    uint8_t stppx;
    uint8_t cStppx;
    uint8_t dir;
    uint16_t fsSteps;
    uint16_t rampSteps;
    uint16_t homeOffset;
    const uint16_t *profile;
    int16_t position;
    stepper_state_t busy;
} axis;

typedef union {
    uint8_t value;
    struct {
        uint8_t xHome  :1,
                yHome  :1,
                zHome  :1,
                eStop  :1,
                unused :4;
    };
} switches_t;

void stepperInit (void);
void homeX (void);
void homeY (void);
void homeZ (void);
void homeAll (void);
bool moveX (const int16_t pixels);
bool moveY (const int16_t steps);
bool moveZ (const int16_t steps);
int16_t getXPos(void);
int16_t getYPos(void);
int16_t getZPos(void);
void zeroAll (void);
void stopAll (void);
void startX (void);
void stopX (void);
void setXHomePos (uint16_t homeOffset);
void setYHomePos (uint16_t homeOffset);
void setXBacklashComp (uint16_t steps);
void toggleXDir (void);
void enableMotors (bool on);
switches_t switch_status (void);
struct axis *calcXParams (const int16_t pixels);
