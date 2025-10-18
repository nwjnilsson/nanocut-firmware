# Intro
This repository was originally forked from
[UnfinishedBusiness/XmotionFirmware](https://github.com/UnfinishedBusiness/XmotionFirmware)
which is a [GRBL](https://github.com/grbl/grbl)-based firmware with support for
adding a Torch Height Controller for CNC plasma cutting. The same author has
also created the control software [NcPilot](https://github.com/UnfinishedBusiness/ncPilot)
(my fork of that can be found [here](https://github.com/nwjnilsson/nanocut-control)),
which is a cross-platform g-code sender compatible with this firmware.

At the moment I don't remember why I had to fork it but for some reason the
original firmware didn't work very well for me. The problem was that the THC
was moving way too slowly. I have refactored the code a bit and broken out the
THC stuff in `thc.c`. I've made some changes so that the THC uses timer 2 to
generate interrupts at a frequency of 10 KHz to generate the step pulses.
When the THC step pulse goes high, it busy waits for `THC_PULSE_TIME_US`
microseconds before setting the pin low again. It works well enough for my use
case. Do _not_ expect this to work if you need very high pulse frequencies, i.e
if your desired `feedrate(mm/min) * steps_per_mm` is some number much higher
than ~120k (what I tested with). If you need more speed, experiment with
gear/pulley ratios or optimize `ISR(TIMER2_OVF_vect)` to reach higher pulse
frequencies.

**tldr:** For a regular Nema23 on the Z axis, stay below a microstepping factor
of 4, or disable it entirely to make the THC as fast as possible.

## Notes regarding the THC
I have modified the original implementation of the THC but the solution is still
kind of hacky. To get around GRBL's internal control of the steppers, step
pulses for the Z-axis are generated manually using overflow interrupts on timer 2.
To keep the code simple, certain assumptions about the hardware are made.

- The extent of the Z axis must be in the range $[-Z,0]$
- ~~The THC does not care about acceleration values set for the Z axis in machine
parameters. You need to take this into consideration when tuning your speeds and
selecting the stepper for your Z axis.~~
- Acceleration support has been added but not extensively tested. Make sure to set
reasonable values... To fit the code on the Nano I removed code related to coolant
control from grbl. I don't see it being used for plasma cutters anyway. When a Z
axis limit is reached, the pulses will stop abruptly, as I didn't have time or
energy to continue refining the implementation. But for simple torch height control,
this should be more than good enough.


## Getting GRBL to run on the Arduino Nano
The config files in this repository are made for using an Arduino Nano
(although the configs can easily be changed). I used the [Minicore](https://github.com/MCUdude/MiniCore)
bootloader since the whole application wouldn't fit in the flash memory with
the standard bootloader. Flashing the bootloader can be done by using the
Arduino IDE and a spare Arduino as a programmer. The application itself was
built and uploaded to the board using [PlatformIO](https://platformio.org/).

# Pinout
My specific pinout config (`CPU_MAP_PLASMA_NANO` in `cpu_map.h`) can be found below. I'm running a dual-Y stepper setup, so I will refer to the two as Y1 and Y2. 

**IMPORTANT**\
If you come across this repo and want to use the code for your machine, make sure to go through `config.h` and understand the implications of all the defines, as it is currently configured for my specific machine. The NanoCut Control software supports configuring limit pin inversion (i.e the meaning of high/low), but not changing the pull-up configuration. 

**Note** D13 is unused because I encountered issues when trying to use it as the Y2 limit. Unsure if pin is used by something else and there is a conflict.
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
| D13 | Unused |
| A0 | Control reset |
| A1 | Torch enable |
| A2 | Arc OK |
| A3 | Y2 dir |
| A4 | Y2 step |
| A5 | Probe (touch torch)  |
| A6 | Cycle start (unused) |
| A7 | Arc voltage (0-5V) |
