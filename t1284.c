#define F_CPU 20000000UL

#include <avr/io.h>
#include <util/delay.h>

#define LED PB4

int main(void)
{
   // Set Pin 5 (PB4) as an output pin.
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
