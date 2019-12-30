# Makefile for simple AVR programming

# We'll pick up the GCC toolchain from the Arduino installation
ARDUINO=/home/john/Arduino/arduino-1.8.10

MCU45=attiny45
MCU23=attiny2313
MCU1616=attiny1616
MCU328=atmega328p
MCU1284=atmega1284p

CC=$(ARDUINO)/hardware/tools/avr/bin/avr-gcc
LD=$(ARDUINO)/hardware/tools/avr/bin/avr-gcc
DUDE=$(ARDUINO)/hardware/tools/avr/bin/avrdude

CFLAGS=-c -o $@ -O3
LDFLAGS=-o $@
DUDEFLAGS=-C $(ARDUINO)/hardware/tools/avr/etc/avrdude.conf

# Programming port and programming device. Can be overridden from the
# command line, e.g. make ISPPORT=/dev/ttyUSB0 prog45
ISPPORT=/dev/ttyS4
ISPDEV=avrispv2
UPDIPORT=/dev/ttyUSB0
UPDIDEV=jtag2updi

OBJS=t45.o t2313.o t1616.o t328p.o t1284p.o
ELFS=$(OBJS:.o=.elf)

# Default target will compile and link all C sources, but not program anything
all: $(ELFS)
.PHONY: all

t45.elf: t45.o
	$(LD) -mmcu=$(MCU45) $(LDFLAGS) t45.o

t45.o: t45.c
	$(CC) -mmcu=$(MCU45) $(CFLAGS) t45.c

t2313.elf: t2313.o
	$(LD) -mmcu=$(MCU23) $(LDFLAGS) t2313.o

t2313.o: t2313.c
	$(CC) -mmcu=$(MCU23) $(CFLAGS) t2313.c

t1616.elf: t1616.o
	$(LD) -mmcu=$(MCU1616) $(LDFLAGS) t1616.o

t1616.o: t1616.c
	$(CC) -mmcu=$(MCU1616) $(CFLAGS) t1616.c

t328p.elf: t328p.o
	$(LD) -mmcu=$(MCU328) $(LDFLAGS) t328p.o

t328p.o: t328p.c
	$(CC) -mmcu=$(MCU328) $(CFLAGS) t328p.c

t1284p.elf: t1284p.o
	$(LD) -mmcu=$(MCU1284) $(LDFLAGS) t1284p.o

t1284p.o: t1284p.c
	$(CC) -mmcu=$(MCU1284) $(CFLAGS) t1284p.c

# Targets to invoke the programmer and program the flash memory of the MCU
prog45: t45.elf
	$(DUDE) $(DUDEFLAGS) -c $(ISPDEV) -P $(ISPPORT) -p $(MCU45) -e -U flash:w:t45.elf:e

prog23: t2313.elf
	$(DUDE) $(DUDEFLAGS) -c $(ISPDEV) -P $(ISPPORT) -p $(MCU23) -e -U flash:w:t2313.elf:e

prog1616: t1616.elf
	$(DUDE) $(DUDEFLAGS) -c $(UPDIDEV) -P $(UPDIPORT) -p $(MCU1616) -e -U flash:w:t1616.elf:e

prog328: t328p.elf
	$(DUDE) $(DUDEFLAGS) -c $(ISPDEV) -P $(ISPPORT) -p $(MCU328) -e -U flash:w:t328p.elf:e

prog1284: t1284p.elf
	$(DUDE) $(DUDEFLAGS) -c $(ISPDEV) -P $(ISPPORT) -p $(MCU1284) -e -U flash:w:t1284p.elf:e

.PHONY: prog45 prog23 prog1616 prog328 prog1284

# Targets 'fuse23' and 'fuse328' will set fuses for clock sources
fuse23:
	$(DUDE) $(DUDEFLAGS) -c $(ISPDEV) -P $(ISPPORT) -p $(MCU23) -U lfuse:w:0xE4:m

fuse328:
	$(DUDE) $(DUDEFLAGS) -c $(ISPDEV) -P $(ISPPORT) -p $(MCU328) -U lfuse:w:0xFF:m

.PHONY: fuse23 fuse328

# Targets 'testisp' and 'testupdi' will connect to the programmer and
# read the device ID, but not program it
testisp:
	$(DUDE) $(DUDEFLAGS) -c $(ISPDEV) -P $(ISPPORT) -p $(MCU45)

testupdi:
	$(DUDE) $(DUDEFLAGS) -c $(UPDIDEV) -P $(UPDIPORT) -p $(MCU1616)

.PHONY: testisp testupdi

# Target 'clean' will delete all object files and ELF files
clean:
	-rm $(OBJS) $(ELFS)
