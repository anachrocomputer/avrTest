#define F_CPU (20000000)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED PIN1_bm     // LED on PA1

#define LED_R PIN0_bm   // Red LED on PB0
#define LED_G PIN1_bm   // Green LED on PB1
#define LED_B PIN2_bm   // Blue LED on PB2

volatile uint32_t Milliseconds = 0UL;

ISR(USART0_RXC_vect)
{
}

ISR(USART0_DRE_vect)
{
}

ISR(USART0_TXC_vect)
{
}

ISR(TCA0_OVF_vect)
{
   TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
   Milliseconds++;
   PORTA.OUTTGL = PIN2_bm;
}


uint32_t millis(void)
{
   uint32_t ms;
   
   cli();
   ms = Milliseconds;
   sei();
   
   return (ms);
}

void dally(const uint32_t ticks)
{
   const uint32_t end = millis() + ticks;
   
   while (millis() < end)
      ;
}


int main(void)
{
   _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc); // Select 20MHz RC oscillator
   
   //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm); // Divide-by-six
   _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc); // No divide-by-six

   PORTA.DIR = 0xFF; // Port A to all outputs
   PORTB.DIR = PIN0_bm | PIN1_bm | PIN2_bm;  // Just PB0, PB1, PB2 to outputs
   PORTC.DIR = 0;

   PORTA.OUT = 0xFF;
   PORTB.OUT = LED_R;
   PORTC.OUT = 0xFF;

   TCA0.SINGLE.PER = 1250 - 1;
   TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc;
   TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;
   TCA0.SINGLE.CTRLC = 0;
   TCA0.SINGLE.CTRLD = 0;
   TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
   TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
   
   sei();
   
   while (1) {
      PORTA.OUTSET = LED;        // LED on PA1 on
      PORTB.OUTSET = LED_G;
      
      dally(500UL);

      PORTA.OUTCLR = LED;        // LED on PA1 off
      PORTB.OUTCLR = LED_G;
      
      dally(500UL);
   }
}
