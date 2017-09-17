# PCBLaserFirmvare
Firmware for Laser PCB Exposer based on a MSP430G2553 MCU

Laser PCB Exposer is blu-ray laser "printer" for exposing photo-sensitive PCBs at 1200 dpi resolution by raster-rendering.

A typical workflow is to "print" the PCB artwork to a bitmap, BMP or PNG are supported, then drop the image onto the desktop application for exposing the PCB. I am using the [KiCad EDA](http://kicad-pcb.org/) for my designs, from that "plotting" to PDF and printing the resulting file to a PNG-file via the [PDFCreator](http://www.pdfforge.org/pdfcreator) printer-driver. I prefer to crop the PNG-files before using them for exposing the PCBs, this is not necesseary to do as the exposer software wil automatically do that - but it will speed up the process.

It is possible to have a drill-ready PCB with solder mask in just two hours from outputting the artwork, this includes about one hour for "hardening" the solder mask.

Further information and some images can be found over at [43oh.com](http://forum.43oh.com/topic/9645-pcb-laser-exposerprinter/#comment-72756)

Project can now be directly imported into TI's cloud based IDE [dev.ti.com](https://dev.ti.com)
