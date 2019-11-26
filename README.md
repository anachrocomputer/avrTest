# avrTest

Some simple AVR test programs to verify toolchain, programmer and chip.

The programs are in C and may be compiled with 'avr-gcc' on Linux.
They generate ELF output files which are suitable to program into
AVR chips for testing purposes.

The 'Makefile' also has targets for the AVR programming tool 'avrdude'.

## Chips Supported

At present, there's only support for the ATtiny45, the ATtiny2313,
and the ATmega328P.
I intend to add support for the ATmega1284P and the ATtiny1616 very soon.

## AVR Toolchain

The programs have been compiled, linked and tested using a Linux version
of the 'avr-gcc' toolchain.
This can be installed directly or as part of the Arduino IDE.

## AVR Programmers

I have tested the code with two AVR programmers, an old AVRISP V2,
and a 'jtag2updi' implemented on an ATmega328P.

Note that my AVRISP is an RS-232 interfaced AVRISP V2
(connected to a non-USB serial port),
not the newer USB interfaced AVRISP MKII.
The difference is subtle but important when specifying the parameters
to 'avrdude'
(but easily changed in the 'Makefile').

## Test Setup

Blinking LEDs, of course!
All the LEDs should blink at 1Hz (500ms on, 500ms off).
