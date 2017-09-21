# PCBLaserFirmvare
Firmware for my [Laser PCB Exposer](https://github.com/terjeio/PCBLaserSchematics) based on a MSP430G2553 MCU

---

#### Command set:

**?** - reports version
	
**XPix:\<pixels\>** - sets X size of PCB to \<pixels\> number of pixels
	
NOTE: the X size MOD 7 must be 0, ie. padding must be added to satisfy the 7 pixels per byte format.

**YPix:\<pixels\>** - sets Y size of PCB to \<pixels\> number of pixels

**Start** - starts rendering, rendering data format is 7 pixels per byte offset by 0x3B, 0x1A in the stream cancels rendering. Each row starts with row number in ASCII format followed by a semicolon, then the rederering data terminated by 0x10.

**MoveX:\<pixels\>** - moves X-axis \<pixels>\, 1 step = 1 pixel

**MoveY:\<pixels\>** - moves Y-axis \<pixels>\, 1 step = 1 pixel for belt drive - may be more steps if ballscrew drive

**MoveZ:\<steps\>** - moves Z-axis (focus) \<steps>\, not supported by MKII hardware

**Laser:\<0|1\>** - 0 = laser off, 1 = laser on

**Power:\<n\>** - set laser current, n = 0-4095 \(a 12-bit DAC is used\)

**ZeroAll** - resets internal position counters \(X, Y, Z\) to 0

**HomeXY** - homes X and Y axes

**HomeX** - homes X axis

**HomeY** - homes Y axis

**HomeZ** - homes Z axis, not supported by MKII hardware

**Echo:\<0|1\>** - 0 = echo off, 1 = echo on

**XHome:\<pixels\>** - start offset for X-axis related to home position

**YHome:\<pixels\>** - start offset for Y-axis related to home position

**XBComp:\<pixels\>** - set backlash compensation to number of \<pixels\>

1 pixel @ 1200 DPI is ~21.2 µm, a little less than 1 mil (25.4 µm)

Communication is via serial @ 38400 baud, 8 bits, 1 stop bit, no parity and with hardware handshake \(RTS/CTS\)

---

This project can be directly imported into TI's cloud based IDE available at [dev.ti.com](https://dev.ti.com), there is no need to download and set up a local development environment in order to compile and upload code.

A [MSP-EXP430G2 LaunchPad](http://www.ti.com/tool/MSP-EXP430G2) can be used as a programmer, remove the MCU from the launchpad and connect P5 \(PGM\) to the corresponding pins on the launchpad.

**NOTE:** for the MKII revision the LaunchPad itself can be used as the controller.