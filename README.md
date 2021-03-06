# avrTest

Some simple AVR test programs to verify toolchain, programmer and chip.

The programs are in C and may be compiled with 'avr-gcc' on Linux.
They generate ELF output files which are suitable to program into
AVR chips for testing purposes.

The 'Makefile' also has targets for the AVR programming tool 'avrdude'.

## Chips Supported

At present, there's only support for the ATtiny45, the ATtiny2313,
the ATtiny1616, the ATmega4809, the ATmega328P and the ATmega1284P.
The main reason for this choice of chips is that I have dev boards
for those chips that I can use for testing.

## AVR Toolchain

The programs have been compiled, linked and tested using a Linux version
of the 'avr-gcc' toolchain.
This can be installed directly or as part of the Arduino IDE.

The compiler, linker and programmers are invoked from the Makefile in
the usual way.
Various parameters in the Makefile may be altered to suit the development
setup, e.g. the type of programmers used and the ports that they connect to.
The full pathname to the toolchain is also configured in the Makefile.

Special targets in the Makefile are provided to invoke the programming
device(s) and write the ELF files into the Flash memory in the chips.
These targets are called 'prog45', 'prog23', 'prog1616', 'prog328',
'prog4809', and 'prog1284'.
The 'prog1616' and 'prog4809' targets invoke the UPDI programmer;
all the others invoke the ISP programmer.

There's a Makefile target called 'clean' that deletes the object code files
and the ELF binary files.
It leaves the source code files untouched, of course.

## AVR Programmers

I have tested the code with two AVR programmers:
an old AVRISP V2,
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
This frequency may be measured as a means of verifying correct
clocking of the AVR chip.

On the ATtiny2313 and ATmega328P, an RGB LED should switch
between colours, including fully off and fully on (white).

On the ATmega1284P, an RGB LED on the PWM pins should fade between colours.
In addition, a 500Hz square wave should be generated on pin PB1.

On the ATtiny1616, an RGB LED on the PWM pins should fade between colours
while another RGB LED on GPIO pins of PORTB will switch between colours.
In addition, a 500Hz square wave should be generated on pin PA4.

On the ATmega4809, an RGB LED on the PWM pins should fade between colours
while another RGB LED on GPIO pins of PORTD will switch between colours.
In addition, a 500Hz square wave should be generated on pin PA4.

On the ATtiny2313, ATtiny1616, ATmega328P and ATmega1284P,
the serial port(s) should transmit a message at 9600 baud.
In the case of the ATmega1284P, a slightly different message will
appear on UART1.
The ATmega4809 also produces a different message on each of its four
UARTs, but with a DIP-40 chip UART3 is inaccessible (only the 48-pin
surface-mount versions can use all four UARTs).

On the ATmega1284P, ATtiny1616 and ATmega4809,
serial input is accepted on UART0.
All the chips accept a letter 'r' to print the reset reason.
The ATtiny1616 and ATmega4809 also accept 'i' to print the chip ID
bytes, 'n' to print the unique serial number and 'f' to print the
values of the fuse registers.

## Future Enhancements

* Test with other AVR chips (e.g. other 1-series ATtiny chips)
* Test with 48-pin SMD ATmega4809
* Add support for 2-series AVRs (e.g. ATtiny1626)
* Add support for TPI (Tiny Programming Interface) chips (e.g. ATtiny20)
* Add support for the very small ATtiny10 in SOT23-6 package
* Configure and test the watchdog timer
