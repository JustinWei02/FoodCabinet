# MCCI's version of Adafruit I2C FRAM Driver

This driver is adapted from Adafruit's driver for the
Adafruit I2C FRAM breakout. https://www.adafruit.com/products/1895

MCCI fixed bugs, and added support for 2K FRAM modules.

## About this Driver

These modules use I2C to communicate, 2 pins are required to
interface

Adafruit and MCCI invest time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit and MCCI!

Written by Kevin (KTOWN) Townsend for Adafruit Industries.
BSD license, check license.txt for more information
All text above must be included in any redistribution

To download, click the ZIP button in the top bar, and check this tutorial
for instructions on how to install:
http://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use

## Compatibility

MCU                | Tested Works | Doesn't Work | Not Tested  | Notes
------------------ | :----------: | :----------: | :---------: | -----
ATSAM21D           |      X       |              |             | Adafruit Feather M0 + MCCI 4450, 4460, etc. Feather Wings
STM32L0            |      X       |              |             | Murata Murata CMWX1ZZABZ-078 on [MCCI Catena 4610](https://mcci.io/catena4610), [4612](https://mcci.io/catena4612), [4801](https://mcci.io/catena4801), etc.
