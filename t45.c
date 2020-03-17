#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED PB4

uint8_t SavedMCUSR = 0;
volatile uint32_t Milliseconds = 0UL;
volatile uint8_t Tick = 0;


/* TIMER1_COMPA_vect --- ISR for Timer/Counter 1 overflow, used for 1ms ticker */

ISR(TIMER1_COMPA_vect)
{
   Milliseconds++;
   Tick = 1;
   PINB = (1 << PB2);         // DEBUG: 500Hz on PB2 pin
}


/* millis --- return milliseconds since reset */

uint32_t millis(void)
{
   uint32_t ms;
   
   cli();
   ms = Milliseconds;
   sei();
   
   return (ms);
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
   SavedMCUSR = MCUSR;
   MCUSR = 0;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   // Set Pin 3 (PB4) and Pin 6 (PB1) as output pins
   DDRB |= (1 << LED) | (1 << PB2) | (1 << PB1) | (1 << PB0);
   PORTB = 0;  // ALl LEDs off
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
   // Config Timer 0 for PWM
   TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
   TCCR0B = (1 << CS01);   // Clock source = CLK/8, start PWM
   OCR0A = 0x80;
   OCR0B = 0x80;
#if 0
   // Config Timer 1 for PWM
   TCCR1A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
   TCCR1B = (1 << CS01);   // Clock source = CLK/8, start PWM
   OCR1A = 0x80;
   OCR1B = 0x80;
#endif
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up Timer/Counter 1 for regular 1ms interrupt
   TCCR1 = (1 << CTC1) | (1 << CS11) | (1 << CS10); // Prescaler /4
   OCR1A = 249;                  // 250 counts gives 1ms
   TCNT1 = 0;
   TIMSK = (1 << OCIE1A);        // Enable interrupts
}


int main(void)
{
   initMCU();
   initGPIOs();
   initPWM();
   initMillisecondTimer();

   sei();   // Enable interrupts

   while (1) {
      // Switch LED on
      PORTB |= 1 << LED;

      OCR0A += 16;
      OCR0B -= 16;
      
      _delay_ms(500);

      // Switch LED off
      PORTB &= ~(1 << LED);
      
      OCR0A += 16;
      OCR0B -= 16;

      _delay_ms(500);
   }
}
