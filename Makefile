# Makefile for simple AVR programming

MCU45=attiny45
MCU23=attiny2313
MCU328=atmega328p
CC=avr-gcc
LD=avr-gcc
PORT=/dev/ttyS4

all: t45.elf t2313.elf t328p.elf

t45.elf: t45.o
	$(CC) -mmcu=$(MCU45) -o t45.elf t45.o

t45.o: t45.c
	$(LD) -mmcu=$(MCU45) -o t45.o -O3 -c t45.c

t2313.elf: t2313.o
	$(CC) -mmcu=$(MCU23) -o t2313.elf t2313.o

t2313.o: t2313.c
	$(LD) -mmcu=$(MCU23) -o t2313.o -O3 -c t2313.c

t328p.elf: t328p.o
	$(CC) -mmcu=$(MCU328) -o t328p.elf t328p.o

t328p.o: t328p.c
	$(LD) -mmcu=$(MCU328) -o t328p.o -O3 -c t328p.c

prog45: t45.elf
	avrdude -c avrispv2 -P $(PORT) -p $(MCU45) -e -U flash:w:t45.elf:e

prog23: t2313.elf
	avrdude -c avrispv2 -P $(PORT) -p $(MCU23) -e -U flash:w:t2313.elf:e

prog328: t328p.elf
	avrdude -c avrispv2 -P $(PORT) -p $(MCU328) -e -U flash:w:t328p.elf:e

fuse23:
	avrdude -c avrispv2 -P $(PORT) -p $(MCU23) -U lfuse:w:0xE4:m

fuse328:
	avrdude -c avrispv2 -P $(PORT) -p $(MCU328) -U lfuse:w:0xFF:m

test:
	avrdude -c avrispv2 -P $(PORT) -p $(MCU45)

