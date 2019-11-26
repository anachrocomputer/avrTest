/* t1616 --- test code for ATtiny1616 and other 1-series chips 2019-11-26 */

#define F_CPU (20000000)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED PIN1_bm     // LED on PA1

#define LED_R PIN0_bm   // Red LED on PB0
#define LED_G PIN1_bm   // Green LED on PB1
#define LED_B PIN2_bm   // Blue LED on PB2

volatile uint32_t Milliseconds = 0UL;
volatile uint8_t Tick = 0;

ISR(USART0_RXC_vect)
{
}

ISR(USART0_DRE_vect)
{
}

ISR(USART0_TXC_vect)
{
}


/* TCB0_OVF_vect --- ISR for Timer/Counter 0 overflow, used for 1ms ticker */

ISR(TCB0_INT_vect)
{
   TCB0.INTFLAGS = TCB_CAPT_bm;
   Milliseconds++;
   Tick = 1;
   PORTA.OUTTGL = PIN2_bm;    // DEBUG: 500Hz on PA2 pin
}


uint32_t millis(void)
{
   uint32_t ms;
   
   cli();
   ms = Milliseconds;
   sei();
   
   return (ms);
}


/* setRGBLed --- control RGB LED connected to PORT B */

void setRGBLed(const int state, const uint8_t fade)
{
   switch (state) {
   case 0:                    // Red fading up, blue on
      TCA0.SINGLE.CMP0 = fade;
      TCA0.SINGLE.CMP1 = 0;
      TCA0.SINGLE.CMP2 = 255;
      PORTB.OUTSET = LED_R;
      PORTB.OUTCLR = LED_G;
      PORTB.OUTCLR = LED_B;
      break;
   case 1:                    // Red on, blue fading down
      TCA0.SINGLE.CMP0 = 255;
      TCA0.SINGLE.CMP1 = 0;
      TCA0.SINGLE.CMP2 = 255 - fade;
      PORTB.OUTSET = LED_R;
      PORTB.OUTSET = LED_G;
      PORTB.OUTCLR = LED_B;
      break;
   case 2:                    // Red on, green fading up
      TCA0.SINGLE.CMP0 = 255;
      TCA0.SINGLE.CMP1 = fade;
      TCA0.SINGLE.CMP2 = 0;
      PORTB.OUTCLR = LED_R;
      PORTB.OUTSET = LED_G;
      PORTB.OUTCLR = LED_B;
      break;
   case 3:                    // Red fading down, green on
      TCA0.SINGLE.CMP0 = 255 - fade;
      TCA0.SINGLE.CMP1 = 255;
      TCA0.SINGLE.CMP2 = 0;
      PORTB.OUTCLR = LED_R;
      PORTB.OUTSET = LED_G;
      PORTB.OUTSET = LED_B;
      break;
   case 4:                    // Green on, blue fading up
      TCA0.SINGLE.CMP0 = 0;
      TCA0.SINGLE.CMP1 = 255;
      TCA0.SINGLE.CMP2 = fade;
      PORTB.OUTCLR = LED_R;
      PORTB.OUTCLR = LED_G;
      PORTB.OUTSET = LED_B;
      break;
   case 5:                    // Green fading down, blue on
      TCA0.SINGLE.CMP0 = 0;
      TCA0.SINGLE.CMP1 = 255 - fade;
      TCA0.SINGLE.CMP2 = 255;
      PORTB.OUTSET = LED_R;
      PORTB.OUTCLR = LED_G;
      PORTB.OUTSET = LED_B;
      break;
   }
}


int main(void)
{
   int ledState = 0;
   uint8_t fade = 0;
   uint32_t end;
   
   _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc); // Select 20MHz RC oscillator
   
   //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm); // Divide-by-six
   _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc); // No divide-by-six

   PORTA.DIR = 0xFF; // Port A to all outputs
   PORTB.DIR = PIN0_bm | PIN1_bm | PIN2_bm;  // Just PB0, PB1, PB2 to outputs
   PORTC.DIR = 0;

   PORTA.OUT = 0xFF;
   PORTB.OUT = LED_R;
   PORTC.OUT = 0xFF;

   // Set up TCA0 for three PWM outputs
   TCA0.SINGLE.PER = 255;
   TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc;
   TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm;
   TCA0.SINGLE.CTRLC = 0;
   TCA0.SINGLE.CTRLD = 0;
   TCA0.SINGLE.CMP0 = 0;   // Red PWM
   TCA0.SINGLE.CMP1 = 0;   // Green PWM
   TCA0.SINGLE.CMP2 = 16;  // Blue PWM
   TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
   
   // Set up TCB0 for regular 1ms interrupt
   TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc;
   TCB0.CTRLB = TCB_CNTMODE_INT_gc;
   TCB0.CCMP = 9999;             // 10000 counts gives 1ms
   TCB0.CNT = 0;
   TCB0.INTCTRL = TCB_CAPT_bm;   // Enable interrupts
   TCB0.CTRLA |= TCB_ENABLE_bm;  // Enable timer
   
   sei();   // Enable interrupts
   
   end = millis() + 500UL;
   
   while (1) {
      if (Tick) {
         if (fade == 255) {
            fade = 0;

            if (ledState == 5)
               ledState = 0;
            else
               ledState++;
         }
         else
            fade++;
            
         setRGBLed(ledState, fade);

         if (millis() > end) {
            PORTA.OUTTGL = LED;        // LED on PA1 toggle
            end = millis() + 500UL;
         }
         
         Tick = 0;
      }
   }
}
