#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED PB0

#define LED_R PB2
#define LED_G PB3
#define LED_B PB4

#define BAUDRATE (9600)
#define BAUD_SETTING ((F_CPU / (BAUDRATE * 16UL)) - 1)

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


void t1ou(const int ch)
{
   while ((UCSRA & (1 << UDRE)) == 0)
      ;
      
   UDR = ch;
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
   // Set up output pins
   DDRB |= (1 << LED) | (1 << LED_R) | (1 << LED_G) | (1 << LED_B) | (1 << PB1);
   PORTB = 0;  // ALl LEDs off
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   // Set baud rate
   UBRRH = (uint8_t)(BAUD_SETTING >> 8); 
   UBRRL = (uint8_t)(BAUD_SETTING);
   // Enable receive and transmit
   UCSRB = (1 << RXEN) | (1 << TXEN);
   // Set frame format
   UCSRC = (1 << UCSZ0) | (1 << UCSZ1);  // Async 8N1
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
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
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up Timer/Counter 1 for regular 1ms interrupt
   TCCR1A = 0;             // WGM11 and WGM10 are set to 0 for CTC mode
   TCCR1B = (1 << WGM12) | (1 << CS10);   // WGM13 set to 0 and WGM12 set to 1 for CTC mode
                                          // CS10 set to 1 for divide-by-1 prescaler
   OCR1A = 7999;                 // 8000 counts gives 1ms
   TCNT1 = 0;
   TIMSK = (1 << OCIE1A);        // Enable interrupts
}


int main(void)
{
   int i = 0;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initPWM();
   initMillisecondTimer();

   sei();   // Enable interrupts

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
