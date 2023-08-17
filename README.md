# Intro
This repository was originally forked from [UnfinishedBusiness/XmotionFirmware](https://github.com/UnfinishedBusiness/XmotionFirmware) which is a [GRBL](https://github.com/grbl/grbl)-based firmware with support for adding a Torch Height Controller for CNC plasma cutting. The same author has also created the control software [NcPilot](https://github.com/UnfinishedBusiness/ncPilot) which is a cross-platform g-code sender compatible with this firmware.

# Getting GRBL to run on the Arduino Nano
The config files in this repository are made for using an Arduino Nano (although the configs can easily be changed). I used the [Minicore](https://github.com/MCUdude/MiniCore) bootloader since the whole application wouldn't fit in the flash memory with the standard bootloader. Flashing the bootloader can be done by using the Arduino IDE and a spare Arduino as a programmer. The application itself was built and uploaded to the board using [PlatformIO](https://platformio.org/).

# Pinout
My specific pinout config can be found below. I'm running a dual-Y stepper setup, so I will refer to the two as Y1 and Y2. 

| Pin | Function |
|----|----|
| D2 | X step |
| D3 | Y1 step |
| D4 | Z step |
| D5 | X dir |
| D6 | Y1 dir |
| D7 | Z dir |
| D8 | Stepper driver enable/disable |
| D9 | X limit |
| D10 | Y1 limit |
| D11 | Z limit |
| D12 | Y2 limit |
| D13 | Spindle/Torch enable |
| A0 | Arc voltage (0-5V) |
| A1 | Arc OK |
| A2 | Cycle start |
| A3 | Y2 dir |
| A4 | Y2 step |
| A5 | Probe (touch torch) |
| A6 | Coolant flood (not used) |
| A7 | Unused |
