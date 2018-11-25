//
// config.h - port assignments etc for PCB laser
//
// v1.5 / 2018-07-01 / Io Engineering / Terje
//

/*

Copyright (c) 2018, Terje Io
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

#include "portmacros.h"

#define LAUNCHPAD  // comment out if for custom hardware
//#define HAS_DAC    // comment out if no DAC (for laser power) is available
#define XSTPPIXELS 1
#define YSTPPIXELS 1

// The original exposer was built around a KR33 linear actuator as the Y-axis drive
// This needed multiple steps per pixel in the Y-direction
//#define THK_KR33_Y // Uncomment if using the THK linear actuator

#ifdef __MSP430F5310__

// MSP430F5310 based custom board

#define SERIAL_PORT   4
#define RXD           BIT4 // P4.4
#define TXD           BIT5 // P4.5
#define RTS_PORT      5
#define RTS_BIT       BIT0 // P5.0
#define SERIAL_MODULE A1

#define I2C_PORT   4
#define SDC        BIT2 // P4.2
#define SDA        BIT1 // P4.1
#define I2C_MODULE B1

#define SERIAL_TXIE  UCTXIE
#define SERIAL_RXIE  UCRXIE
#define SERIAL_TXIFG UCTXIFG
#define SERIAL_RXIFG UCRXIFG

#define XSTEP_TIMER  A
#define XSTEP_TIMERI 1
#define XSTEP_PORT   2
#define XSTEP_BIT    BIT0 // P2.0
#define XDIR_PORT    1
#define XDIR_BIT     BIT3 // P1.3
#define XMOT_PORT    J
#define XMOT_BIT     BIT1 // PJ.3
#define XHOME_PORT   6
#define XHOME_BIT    BIT1 // P6.3

#define YSTEP_TIMER  A
#define YSTEP_TIMERI 0
#define YSTEP_PORT   1
#define YSTEP_BIT    BIT2 // P1.2
#define YDIR_PORT    1
#define YDIR_BIT     BIT7 // P1.7
#define YMOT_PORT    J
#define YMOT_BIT     BIT1 // PJ.1
#define YHOME_PORT   6
#define YHOME_BIT    BIT2 // P6.2

#define ZSTEP_PORT 4
#define ZSTEP_BIT  BIT6 // P4.6
#define ZDIR_PORT  4
#define ZDIR_BIT   BIT3 // P4.3
#define ZMOT_PORT  J
#define ZMOT_BIT   BIT1 // PJ.1
#define ZHOME_PORT J
#define ZHOME_BIT  BIT2 // PJ.2

#define ESTOP_PORT 6
#define ESTOP_BIT  BIT3 // P6.3

#define LASER_PORT 4
#define LASER_BIT  BIT7 // P4.7

#define VDAC_PORT J
#define VDAC_BIT  BIT3 // PJ.3

#else // MSP430G2553 LaunchPad or original MSP430G2553 based custom board

#define SERIAL_PORT   1
#define RXD           BIT1  // P1.1
#define TXD           BIT2  // P1.2
#define RTS_PORT      1
#define RTS_BIT       BIT3 // P1.3
#define SERIAL_MODULE A0

#define I2C_PORT   1
#define SDC        BIT6  // P1.6
#define SDA        BIT7  // P1.7
#define I2C_MODULE B0

#define SERIAL_TXIE UCA0TXIE
#define SERIAL_RXIE UCA0RXIE
#define SERIAL_TXIFG UCA0TXIFG
#define SERIAL_RXIFG UCA0RXIFG

#define XSTEP_TIMER  A
#define XSTEP_TIMERI 0
#ifdef HAS_DAC
#define XSTEP_PORT   2
#define XSTEP_BIT    BIT6  // P2.6
#else
#define XSTEP_PORT   1
#define XSTEP_BIT    BIT6  // P1.6
#endif
#ifdef LAUNCHPAD
#define XDIR_PORT    2
#define XDIR_BIT     BIT5 // P2.5
#define XMOT_PORT    2
#define XMOT_BIT     BIT7 // P2.7
#else
#define XDIR_PORT    2
#define XDIR_BIT     BIT7 // P2.7
#endif
#define XHOME_PORT   1
#define XHOME_BIT    BIT0 // P1.0

#define YSTEP_TIMER  A
#define YSTEP_TIMERI 1
#define YSTEP_PORT   2
#define YSTEP_BIT    BIT1 // P2.1
#ifdef LAUNCHPAD
#define YDIR_PORT    2
#define YDIR_BIT     BIT2 // P2.2
#define YMOT_PORT    2
#define YMOT_BIT     BIT0 // P2.0
#define YHOME_PORT   1
#define YHOME_BIT    BIT4 // P1.4
#else
#define YDIR_PORT    3
#define YDIR_BIT     BIT2 // P3.2
#define YHOME_PORT   2
#define YHOME_BIT    BIT0 // P2.0
#endif

#define ZSTEP_PORT   2
#define ZSTEP_BIT    BIT2 // P2.2

#ifdef LAUNCHPAD
#define ZDIR_PORT    2
#define ZDIR_BIT     BIT4 // P2.4
#define ZMOT_PORT    2
#define ZMOT_BIT     BIT0 // P2.0
#define ZHOME_PORT   1
#define ZHOME_BIT    BIT5 // P1.5
#else
#define ZDIR_PORT    3
#define ZDIR_BIT     BIT6 // P3.6
#define ZHOME_PORT   2
#define ZHOME_BIT    BIT4 // P1.5
#endif

#ifdef LAUNCHPAD
#define ESTOP_PORT 1
#define ESTOP_BIT  BIT5 // P1.5
#define LASER_PORT 2
#define LASER_BIT  BIT6 // P2.6
#else
#define ESTOP_PORT 3
#define ESTOP_BIT  BIT0 // P3.0
#define LASER_PORT 2
#define LASER_BIT  BIT5 // P2.5
#endif

#endif // MSP430G2553
