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
   PINB = (1 << PB1);         // DEBUG: 500Hz on PB1 pin
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
   DDRB |= (1 << LED) | (1 << PB1);
   PORTB = 0;  // ALl LEDs off
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
   initMillisecondTimer();

   sei();   // Enable interrupts

   while (1) {
      // Switch LED on
      PORTB |= 1 << LED;

      _delay_ms(500);

      // Switch LED off
      PORTB &= ~(1 << LED);

      _delay_ms(500);
   }
}
