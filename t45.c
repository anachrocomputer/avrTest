#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>

#define LED PB4

uint8_t SavedMCUSR = 0;

int main(void)
{
   SavedMCUSR = MCUSR;
   MCUSR = 0;
   
   // Set Pin 3 (PB4) as an output pin.
   DDRB |= 1 << LED;

   while (1) {
      // Switch LED on
      PORTB |= 1 << LED;

      _delay_ms(500);

      // Switch LED off
      PORTB &= ~(1 << LED);

      _delay_ms(500);
   }
}
