//
// main.c - PCB exposer (for 405nm laser diode) - MSP430G2553
//
// v1.5 / 2018-07-01 / Io Engineering / Terje
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
#include "serial.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "stepper.h"
#include "serial.h"

#define LASER_PORT_IN   portIn(LASER_PORT)
#define LASER_PORT_OUT  portOut(LASER_PORT)
#define LASER_PORT_DIR  portDir(LASER_PORT)
#if LASER_PORT == 2 && (LASER_BIT == BIT6 || LASER_BIT == BIT7)
#define LASER_PORT_SEL  portSel(LASER_PORT)
#define LASER_PORT_SEL2 portSel2(LASER_PORT)
#endif

#define VDAC_PORT_OUT  portOut(VDAC_PORT)
#define VDAC_PORT_DIR  portDir(VDAC_PORT)

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
    "\r\nIo Engineering PCB Laser rev 1.5\0",
    "ESTOP!",
    "Error: missing parameters",
    "Job running...",
    "OK",
    "FAILED",
    "Bad command"
};

typedef struct {
    char const *const command;
    bool (*const handler)(char *);
    const bool report;
} command_t;

const char cr = '\r';
const char lf = '\n';

#ifdef THK_KR33_Y
const char ysteps[] = {6,5,6,5,6,6,6,5,6,5,6,6,6,5,6,5,6,6,6,5,6,5,6,5,6,5,6,5,6,6,6,5,6,5,6,6,6,5,6,5,6,6,6,5,6};
const char ystepn = sizeof(ysteps) / sizeof(ysteps[0]);
#endif

char cmdbuf[16];
uint8_t pxCounter, cancelled = 0, echo = 1, bcomp = 0;
uint16_t xpixels = 0, ypixels = 0;

void exeCommand (char *cmdline);

bool waitForRowStart (uint16_t line)
{
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
                cancelled = 2;
            }
        }
    }

    if(!cancelled && (chr = serialRead()) == EOF)
        cancelled = 3;

    return chr - BYTEOFFSET;
}

bool renderRow (uint16_t x, uint16_t y)
{
    struct axis *axis;
    uint16_t pixels, rows = y;
#ifdef THK_KR33_Y
    uint16_t yseq = 0;
#endif

    if(switch_status().eStop)
        return false;

    enableMotors(true);

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
                LASER_PORT_OUT |= LASER_BIT;
            else
                LASER_PORT_OUT &= ~LASER_BIT;

            if(--pxCounter)
                pixels = pixels >> 1;

            else {

                pxCounter = BYTEPIXELS;

                if((pixels = serialRead()) == EOF) {
                    stopX();
                    cancelled = 4;
                }

                pixels -= BYTEOFFSET;

            }

            if(axis->busy)
                LPM0;

        }

        if(!cancelled) {

            LASER_PORT_OUT |= LASER_BIT;    // Switch off laser

            toggleXDir();                   // and change X direction

#ifdef THK_KR33_Y
            moveY(ysteps[yseq++]);          // Move table one pixel in Y direction
            if(yseq == ystepn)
                yseq = 0;
#else
            moveY(1);                       // Move table one pixel in Y direction
#endif
        }

    }

    if(cancelled)
        serialRxFlush();

    stopAll();

    moveX(-getXPos());              // Move table back
    moveY(-getYPos());              // to origin

    LASER_PORT_OUT |= LASER_BIT;    // switch off laser (ESTOP may have been requested)

    return !switch_status().eStop && !cancelled;
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

#ifdef __MSP430F5310__

void SetVCoreUp (unsigned int level)
{
    // Open PMM registers for write access
    PMMCTL0_H = 0xA5;
    // Set SVS/SVM high side new level
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
    // Set SVM low side to new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
    // Wait till SVM is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0);
    // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
    // Wait till new level reached
    if ((PMMIFG & SVMLIFG))
        while ((PMMIFG & SVMLVLRIFG) == 0);
    // Set SVS/SVM low side to new level
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;
}

#endif

void main (void)
{
    char c;
    uint16_t cmdptr = 0;

    WDTCTL = WDTPW | WDTHOLD;               // Stop watchdog timer

#ifdef __MSP430F5310__

    /* remap some pins */
/*
    PMAPKEYID = PMAPKEY;
    P4MAP7 = PM_UCB1SDA;
    P4MAP6 = PM_UCB1SCL;
    P4MAP2 = PM_TB0CCR1A;
    PMAPKEYID = 0;
*/
    /* Power settings */
     SetVCoreUp(1u);
     SetVCoreUp(2u);
     SetVCoreUp(3u);

     UCSCTL3 = SELREF__REFOCLK;    // select REFO as FLL source
     UCSCTL6 = XT1OFF | XT2OFF;    // turn off XT1 and XT2

     /* Initialize DCO to 25.00MHz */
     __bis_SR_register(SCG0);                  // Disable the FLL control loop
     UCSCTL0 = 0x0000u;                        // Set lowest possible DCOx, MODx
     UCSCTL1 = DCORSEL_5;                      // Set RSELx for DCO = 50 MHz
     UCSCTL2 = 762u;                            // Set DCO Multiplier for 25MHz
                                               // (N + 1) * FLLRef = Fdco
                                               // (762 + 1) * 32768 = 25.00MHz
     UCSCTL4 = SELA__REFOCLK | SELS__DCOCLK | SELM__DCOCLK;
//     UCSCTL4 = SELA__XT1CLK | SELS__DCOCLK | SELM__DCOCLK;
     __bic_SR_register(SCG0);                  // Enable the FLL control loop

     // Worst-case settling time for the DCO when the DCO range bits have been
     // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
     // UG for optimization.
     // 32*32*25MHz/32768Hz = 781250 = MCLK cycles for DCO to settle
     __delay_cycles(781250u);

     /* Loop until XT1,XT2 & DCO fault flag is cleared */
     do {
       UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);  // Clear XT2,XT1,DCO fault flags
       SFRIFG1 &= ~OFIFG;                           // Clear fault flags
     } while (SFRIFG1&OFIFG);                       // Test oscillator fault flag

#else
    DCOCTL = CALDCO_16MHZ;                  // Set DCO for 16MHz using
    BCSCTL1 = CALBC1_16MHZ;                 // calibration registers
#endif

//    P1DIR = 0xFF;                           // All P1 outputs
//    P1OUT = 0;                              // Clear P1 outputs
    LASER_PORT_DIR |= LASER_BIT;            // Enable laser output on P2
#ifdef LASER_PORT_SEL
    LASER_PORT_SEL &= ~LASER_BIT;
#endif
#ifdef LASER_PORT_SEL2
    LASER_PORT_SEL2 &= ~LASER_BIT;
#endif
    LASER_PORT_OUT |= LASER_BIT;            // switch off laser!

#ifdef VDAC_PORT
    VDAC_PORT_DIR |= VDAC_BIT;
    VDAC_PORT_OUT |= VDAC_BIT;
#endif

    //  P3DIR = 0xFF;                           // All P3 outputs
    //  P3OUT = 0;                              // Clear P3 outputs

    serialInit();
    stepperInit();

    _EINT();                                // Enable interrupts

#ifdef HAS_DAC
    initDAC();
#endif

    serialRxFlush();
    serialWriteLn(message[MSG_ABOUT]);
    serialWriteLn(message[switch_status().eStop ? MSG_ESTOP : MSG_OK]);

    while(1) {

        if(serialRxCount()) { // bytes waiting, process them

            c = serialRead();

            if(echo)
                serialPutC(c);

            if(c == cr && cmdptr > 0) {

                cmdbuf[cmdptr] = 0;

                if(echo)
                    serialPutC(lf);

                exeCommand(cmdbuf);

                cmdptr = 0;
                cmdbuf[0] = 0;

            } else if(c == DEL) {
                if(cmdptr > 0)
                    cmdptr--;
            } else if(c > 31 && cmdptr < sizeof(cmdbuf))
                cmdbuf[cmdptr++] = c & (c >= 'a' && c <= 'z' ? 0x5F : 0xFF);
        }
    }
}

bool about (char *params)
{
    serialWriteLn(message[MSG_ABOUT]);
    serialWriteS("OPT:");
#ifdef HAS_DAC
    serialPutC('D');
#endif
#ifdef XMOT_PORT
    serialPutC('M');
#endif
#ifdef HAS_FOCUS
    serialPutC('Z');
#endif
    serialWriteLn("");
    serialWriteLn(message[switch_status().eStop ? MSG_ESTOP : MSG_OK]);
    return true;
}

bool setXPixels (char *params)
{
    xpixels = parseInt(params);
    return true;
}

bool setYPixels (char *params)
{
    ypixels = parseInt(params);
    return true;
}

bool start (char *params)
{
    if(xpixels == 0 || ypixels == 0)
        serialWriteLn(message[MSG_PARAM]);
    else
        serialWriteLn(message[renderRow(xpixels, ypixels) ? MSG_OK : MSG_FAIL]);
    return true;
}

bool moveXaxis (char *params)
{
    enableMotors(true);
    return moveX(parseInt(params));
}

bool moveYaxis (char *params) {
    enableMotors(true);
    return moveY(parseInt(params));
}

bool moveZaxis (char *params) {
    enableMotors(true);
    return moveZ(parseInt(params));
}

bool laser (char *params)
{
    if(parseInt(params) == 0)
        LASER_PORT_OUT |= LASER_BIT;    // switch off laser
    else
        LASER_PORT_OUT &= ~LASER_BIT;   // switch on laser
    return true;
}

bool power (char *params)
{
#ifdef HAS_DAC
    setVoltage(parseInt(params), false);
#endif
    return true;
}

bool zeroAllaxes (char *params)
{
    zeroAll();
    return true;
}

bool homeXYaxses (char *params)
{
    homeX();
    homeY();
    return true;
}

bool homeXaxis (char *params)
{
    homeX();
    return true;
}

bool homeYaxis (char *params)
{
    homeY();
    return true;
}

bool homeZaxis (char *params)
{
    homeZ();
    return true;
}

bool echoMode (char *params)
{
    echo = parseInt(params) != 0;
    return true;
}

bool XHomePos (char *params)
{
    setXHomePos(parseInt(params));
    return true;
}

bool YHomePos (char *params)
{
    setYHomePos(parseInt(params));
    return true;
}

bool XBComp (char *params)
{
    bcomp = parseInt(params);
    return true;
}

bool motorsOn (char *params)
{
    enableMotors(parseInt(params) != 0);
    return true;
}

bool switches (char *params)
{
    switches_t status = switch_status();

    serialWriteS("SW:");

    if(status.eStop)
        serialPutC('E');

    if(status.xHome)
        serialPutC('X');

    if(status.yHome)
        serialPutC('Y');

    if(status.zHome)
        serialPutC('Z');

    serialWriteLn("");

    return true;
}

void exeCommand (char *cmdline)
{
    static const command_t commands[] = {
        "?",        about, false,
        "XPIX:",    setXPixels, false,
        "YPIX:",    setYPixels, false,
        "START",    start, false,
        "MOVEX:",   moveXaxis, true,
        "MOVEY:",   moveYaxis, true,
        "X:",       moveXaxis, true,
        "Y:",       moveYaxis, true,
        "LASER:",   laser, true,
        "ZEROALL",  zeroAllaxes, true,
        "HOMEXY",   homeXYaxses, true,
        "HOMEX",    homeXaxis, true,
        "HOMEY",    homeYaxis, true,
#ifdef HAS_DAC
        "POWER:",   power, true,
#endif
#ifdef HAS_FOCUS
        "MOVEZ:",   moveZaxis, true,
        "Z:",       moveZaxis, true,
        "HOMEZ",    homeZaxis, true,
#endif
        "ECHO:",    echoMode, false,
        "XHOME:",   XHomePos, true,
        "YHOME",    YHomePos, true,
        "XBCOMP:",  XBComp, true,
        "MOTORS:",  motorsOn, true,
        "SWITCHES", switches, false
    };

    static const uint16_t numcmds = sizeof(commands) / sizeof(command_t);

    bool ok = false;
    uint16_t i = 0, cmdlen;

    while(!ok && i < numcmds) {

        cmdlen = strlen(commands[i].command);

        if(!(ok = !strncmp(commands[i].command, cmdline, cmdlen)))
            i++;

    }

    if(ok) {
        ok = commands[i].handler(cmdline + cmdlen);
        if(commands[i].report)
            serialWriteLn(message[ok ? MSG_OK : MSG_FAIL]);
    } else
        serialWriteLn(message[MSG_BADC]);
}
