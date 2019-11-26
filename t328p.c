#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define LED PB0

#define LED_R PB2
#define LED_G PB3
#define LED_B PB4

#define BAUDRATE (9600)
#define BAUD_SETTING ((F_CPU / (BAUDRATE * 16UL)) - 1)

void t1ou(const int ch)
{
   while ((UCSR0A & (1 << UDRE0)) == 0)
      ;
      
   UDR0 = ch;
}

int main(void)
{
   int i = 0;
   
   // Set up output pins
   DDRB |= (1 << LED) | (1 << LED_R) | (1 << LED_G) | (1 << LED_B);
   PORTB = 0;  // ALl LEDs off
   
   // Set baud rate
   UBRR0H = (uint8_t)(BAUD_SETTING >> 8); 
   UBRR0L = (uint8_t)(BAUD_SETTING);
   // Enable receive and transmit
   UCSR0B = (1 << RXEN0) | (1 << TXEN0);
   // Set frame format
   UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);  // Async 8N1

#if 0
   // Config Timer 0 for PWM
   TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
   TCCR0B = (1 << CS01);   // Clock source = CLK/8, start PWM
   OCR0A = 0x80;
   OCR0B = 0x80;
   // Config Timer 1 for PWM
   TCCR1A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
   TCCR1B = (1 << CS01);   // Clock source = CLK/8, start PWM
   OCR1A = 0x80;
   OCR1B = 0x80;
#endif

   t1ou('\r');
   t1ou('\n');
   
   while (1) {
      if (i & 1)
         PORTB |= (1 << LED_R);
      else
         PORTB &= ~(1 << LED_R);
    
      if (i & 2)
         PORTB |= (1 << LED_G);
      else
         PORTB &= ~(1 << LED_G);
    
      if (i & 4)
         PORTB |= (1 << LED_B);
      else
         PORTB &= ~(1 << LED_B);
         
//    OCR0A += 16;
    
      // Switch LED on
      PORTB |= 1 << LED;

      t1ou('U');
      t1ou('U');

      _delay_ms(500);
      
//    OCR0A += 16;

      // Switch LED off
      PORTB &= ~(1 << LED);

      t1ou('A');
      t1ou('B');
      
      _delay_ms(500);
      
      i = (i + 1) & 0x07;
   }
}