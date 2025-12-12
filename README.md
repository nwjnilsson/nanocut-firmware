# Intro
This repository was originally forked from [UnfinishedBusiness/XmotionFirmware](https://github.com/UnfinishedBusiness/XmotionFirmware),
which ~is~ was a [GRBL](https://github.com/grbl/grbl)-based firmware that puts CNC motion control and Torch
Height Control on a single board. The original implementation remained
unfinished and contained several assumptions about the hardware etc that did
not work for me, but it was a good starting point for me to continue building
upon.

My implementation uses timer 2 to generate interrupts at a frequency of 10 KHz
to generate the step pulses. This does not mean that it can reliably output step
pulses at 10 kHz though due to hardware limitations. My guesstimation at the
moment of writing is that you can get good quality pulses of up to about 3 kHz.
You can calculate your needed pulse rate with `feedrate(mm/sec) * steps_per_mm`.
For my machine, 3 kHz is more than enough to drive my Z axis at a microstepping
factor of 4 and a ball screw with a pitch of 4 mm per turn. Experiment with
your microstepping and gear/pulley ratios if you need more speed or by all means
optimize `ISR(TIMER2_OVF_vect)` to reach even higher pulse frequencies.

**tldr:** For a regular Nema23 on the Z axis, stay below a microstepping factor
of 4, or disable it entirely to make the THC as fast as possible.

## Differences from vanilla GRBL
- Torch Height Control, duh
- To fit the code on the Nano I removed code related to coolant
control from grbl. I don't see it being used for plasma cutters anyway.
- New GRBL setting: **$33** sets the arc voltage divider, which the controller will
use to scale the input signal.
- THC target voltage can be set with e.g **\$T=112.5000** when grbl is either
in idle, cycle, or hold state. It is assumed that arc OK is connected to feed
hold.

# Using NanoCut firmware
If you're not already familiar with grbl, you might want to check out some guide
on how to configure it using `cpu_map.h` and `config.h`. Aside from regular
grbl configuration, you'll want to check out `thc.h` to configure it for your
machine. My own pinout config is called `CPU_MAP_PLASMA_NANO` in `cpu_map.h`
(see table below).

**NOTE:** The THC has an acceleration profile for smooth control. This profile does
not, however, apply at the Z limits. Once the axis hits the limit, the stepping
pulses stop abruptly. But if the THC would head for the limits for some reason,
I imagine you have other issues to worry about. Acceleration (or any movement
in general) requires sane parameters and I don't range check much in the
firmware so be cautious and verify that motion behaves well at slow speeds first.
I didn't attach my driving belts at all before making sure the motors were doing
what I wanted.

## Pinout

**Note:** You probably want to avoid using D13 since it's connected to the LED
that the bootloader may flash on/off during startup.

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
| D13 | Unused (see note above) |
| A0 | Control reset |
| A1 | Torch enable |
| A2 | Arc OK |
| A3 | Y2 dir |
| A4 | Y2 step |
| A5 | Probe (touch torch)  |
| A6 | Cycle start (unused) |
| A7 | Arc voltage (0-5V) |


## Getting NanoCut firmware to run on the Arduino Nano
The config files in this repository are made for using an Arduino Nano
(although the configs can easily be changed). I used the [Minicore](https://github.com/MCUdude/MiniCore)
bootloader since the whole application wouldn't fit in the flash memory with
the standard bootloader. Flashing the bootloader can be done by using the
Arduino IDE and a spare Arduino as a programmer. The application itself was
built and uploaded to the board using [PlatformIO](https://platformio.org/).
