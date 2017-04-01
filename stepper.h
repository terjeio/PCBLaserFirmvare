//
// stepper.h - stepper control library, for PCB laser
//
// v1.0 / 2016-06-03 / Io Engineering / Terje
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

#define STEPPULSE 50
#define RAMPSTEPS 200 // includes both leading and trailing ramp!

#define XDIR BIT7	// P2
#define XSTEP BIT6	// P2
#define XHOME BIT0	// P1
#define YSTEP BIT1  // P2 -> P3.2
#define YDIR BIT2   // P3
#define YHOME BIT0	// P2
#define ZDIR BIT6	// P3 -> P2.3
#define ZHOME BIT4	// P2
#define ZSTEP BIT2  // P2
#define ESTOP BIT0  // P3

#define STATE_IDLE  0
#define STATE_ACCEL 1
#define STATE_MOVE  2
#define STATE_DECEL 3

typedef struct axis {
	uint8_t bComp;
	uint8_t cComp;
	uint8_t dirBit;
	uint8_t dir;
	uint16_t fsSteps;
	uint16_t rampSteps;
	uint16_t homeOffset;
	const uint16_t *profile;
	int16_t position;
	uint8_t busy;
} axis;

void stepperInit (void);
bool isESTOP (void);
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
struct axis *calcXParams (const int16_t pixels);
