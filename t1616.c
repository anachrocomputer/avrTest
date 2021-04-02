/* t1616 --- test code for ATtiny1616 and other 1-series chips 2019-11-26 */

#define F_CPU (20000000)

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// UART TxD on PA1 (alternate)
// UART RxD on PA2 (alternate)
#define LED    PIN3_bm  // Blinking LED on PA3
#define SQWAVE PIN4_bm  // 500Hz square wave on PA4

#define LED_R PIN3_bm   // Red LED on PB3
#define LED_G PIN4_bm   // Green LED on PB4
#define LED_B PIN5_bm   // Blue LED on PB5

#define BAUDRATE (9600UL)

#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// UART buffers
struct UART_BUFFER U0Buf;

uint8_t SavedRSTFR = 0;
volatile uint32_t Milliseconds = 0UL;
volatile uint8_t Tick = 0;


/* USART0_RXC_vect --- ISR for USART0 Receive Complete, used for Rx */

ISR(USART0_RXC_vect)
{
   const uint8_t tmphead = (U0Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
   const uint8_t ch = USART0.RXDATAL;  // Read received byte from UART
   
   if (tmphead == U0Buf.rx.tail)   // Is receive buffer full?
   {
       // Buffer is full; discard new byte
   }
   else
   {
      U0Buf.rx.head = tmphead;
      U0Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
   }
}


/* USART0_DRE_vect --- ISR for USART0 Data Register Empty, used for Tx */

ISR(USART0_DRE_vect)
{
   if (U0Buf.tx.head != U0Buf.tx.tail) // Is there anything to send?
   {
      const uint8_t tmptail = (U0Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
      
      U0Buf.tx.tail = tmptail;

      USART0.TXDATAL = U0Buf.tx.buf[tmptail];    // Transmit one byte
   }
   else
   {
      USART0.CTRLA &= ~(USART_DREIE_bm); // Nothing left to send; disable Tx interrupt
   }
}


/* TCB0_OVF_vect --- ISR for Timer/Counter 0 overflow, used for 1ms ticker */

ISR(TCB0_INT_vect)
{
   TCB0.INTFLAGS = TCB_CAPT_bm;
   Milliseconds++;
   Tick = 1;
   PORTA.OUTTGL = SQWAVE;     // DEBUG: 500Hz on PA4 pin
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


/* t1ou --- transmit one character to the UART by polling */

void t1ou(const int ch)
{
   while ((USART0.STATUS & USART_DREIF_bm) == 0)
      ;
      
   USART0.TXDATAL = ch;
}


/* UART0RxByte --- read one character from the UART via the circular buffer */

uint8_t UART0RxByte(void)
{
   const uint8_t tmptail = (U0Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
   
   while (U0Buf.rx.head == U0Buf.rx.tail)  // Wait, if buffer is empty
       ;
   
   U0Buf.rx.tail = tmptail;
   
   return (U0Buf.rx.buf[tmptail]);
}


/* UART0TxByte --- send one character to the UART via the circular buffer */

void UART0TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U0Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U0Buf.tx.tail)   // Wait, if buffer is full
       ;

   U0Buf.tx.buf[tmphead] = data;
   U0Buf.tx.head = tmphead;

   USART0.CTRLA |= USART_DREIE_bm;   // Enable UART0 Tx interrupt
}


/* USART0_printChar --- helper function to make 'stdio' functions work */

static int USART0_printChar(const char c, FILE *stream)
{
   if (c == '\n')
      UART0TxByte('\r');

   UART0TxByte(c);

   return (0);
}

static FILE USART_stream = FDEV_SETUP_STREAM(USART0_printChar, NULL, _FDEV_SETUP_WRITE);


/* UART0RxAvailable --- return true if a byte is available in the UART circular buffer */

int UART0RxAvailable(void)
{
   return (U0Buf.rx.head != U0Buf.rx.tail);
}


/* setRGBLed --- control two RGB LEDs connected to PORT B */

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


/* printDeviceID --- print the Device ID bytes as read from SIGROW */

void printDeviceID(void)
{
   printf("Device ID = %02x %02x %02x\n", SIGROW.DEVICEID0, SIGROW.DEVICEID1, SIGROW.DEVICEID2);
   printf("REVID = %02x\n", SYSCFG.REVID);
}


/* printSerialNumber --- print the chip's unique serial number */

void printSerialNumber(void)
{
   printf("Serial Number = %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
                           SIGROW.SERNUM0, SIGROW.SERNUM1, SIGROW.SERNUM2,
                           SIGROW.SERNUM3, SIGROW.SERNUM4, SIGROW.SERNUM5,
                           SIGROW.SERNUM6, SIGROW.SERNUM7, SIGROW.SERNUM8,
                           SIGROW.SERNUM9);
}


/* printFuses --- print the fuse settings */

void printFuses(void)
{
   printf("FUSES.WDTCFG = 0x%02x\n", FUSE.WDTCFG);
   printf("FUSES.BODCFG = 0x%02x\n", FUSE.BODCFG);
   printf("FUSES.OSCCFG = 0x%02x\n", FUSE.OSCCFG);
   printf("FUSES.TCD0CFG = 0x%02x\n", FUSE.TCD0CFG);
   printf("FUSES.SYSCFG0 = 0x%02x\n", FUSE.SYSCFG0);
   printf("FUSES.SYSCFG1 = 0x%02x\n", FUSE.SYSCFG1);
   printf("FUSES.APPEND = 0x%02x\n", FUSE.APPEND);
   printf("FUSES.BOOTEND = 0x%02x\n", FUSE.BOOTEND);
}


/* printResetReason --- print the cause of the chip's reset */

void printResetReason(void)
{
   printf("RSTCTRL.RSTFR = 0x%02x\n", SavedRSTFR);
}


int getTemp(void)
{
   int8_t sigrow_offset = SIGROW.TEMPSENSE1;  // Read signed value from signature row
   uint8_t sigrow_gain = SIGROW.TEMPSENSE0;    // Read unsigned value from signature row
   
   uint16_t adc_reading = 0;   // ADC conversion result with 1.1 V internal reference 
   uint32_t temp = adc_reading - sigrow_offset;temp *= sigrow_gain;  // Result might overflow 16 bit variable (10bit+8bit)
   temp += 0x80;               // Add 1/2 to get correct rounding on division below
   temp >>= 8;                 // Divide result to get Kelvin 
   uint16_t temperature_in_K = temp;
}


/* initMCU --- set up the microcontroller in general */

static void initMCU(void)
{
   _PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, CLKCTRL_CLKSEL_OSC20M_gc); // Select 20MHz RC oscillator
   
   //_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc | CLKCTRL_PEN_bm); // Divide-by-six
   _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_6X_gc); // No divide-by-six

   SavedRSTFR = RSTCTRL.RSTFR;
   RSTCTRL.RSTFR = RSTCTRL_UPDIRF_bm | RSTCTRL_SWRF_bm | RSTCTRL_WDRF_bm |
                   RSTCTRL_EXTRF_bm | RSTCTRL_BORF_bm | RSTCTRL_PORF_bm;
}


/* initGPIOs --- set up the GPIO pins */

static void initGPIOs(void)
{
   PORTA.DIR = LED | SQWAVE;
   PORTB.DIR = LED_R | LED_G | LED_B;
   PORTC.DIR = 0;

   PORTA.OUT = 0xFF;
   PORTB.OUT = 0xFF;
   PORTC.OUT = 0xFF;
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   // Switch UART pins to the alternate locations to avoid clash with PWM pins
   PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc;

   // Set up UART0 and associated circular buffers
   U0Buf.tx.head = 0;
   U0Buf.tx.tail = 0;
   U0Buf.rx.head = 0;
   U0Buf.rx.tail = 0;

   USART0.BAUD = (F_CPU * 64UL) / (16UL * BAUDRATE);
   USART0.CTRLA = 0;
   USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
   USART0.CTRLA |= USART_RXCIE_bm;   // Enable UART0 Rx interrupt
   USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_RXMODE_NORMAL_gc;
   
   // Enable UART0 TxD pin
   PORTA.DIRSET = PIN1_bm;

   stdout = &USART_stream;    // Allow use of 'printf' and similar functions
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
   // Set up TCA0 for three PWM outputs
   TCA0.SINGLE.PER = 255;
   TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc;
   TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm;
   TCA0.SINGLE.CTRLC = 0;
   TCA0.SINGLE.CTRLD = 0;
   TCA0.SINGLE.CMP0 = 0;   // Red PWM
   TCA0.SINGLE.CMP1 = 0;   // Green PWM
   TCA0.SINGLE.CMP2 = 0;   // Blue PWM
   TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;

   // Enable output on PWM pins
   PORTB.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm;
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up TCB0 for regular 1ms interrupt
   TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc;
   TCB0.CTRLB = TCB_CNTMODE_INT_gc;
   TCB0.CCMP = 9999;             // 10000 counts gives 1ms
   TCB0.CNT = 0;
   TCB0.INTCTRL = TCB_CAPT_bm;   // Enable interrupts
   TCB0.CTRLA |= TCB_ENABLE_bm;  // Enable timer
}


/* initDAC --- set up the 8-bit DAC and connect it to the output pin */

static void initDAC(void)
{
   DAC0.CTRLA = DAC_ENABLE_bm | DAC_OUTEN_bm; // Enable DAC and pin (PA6, pin 4 of SOIC-20)
   VREF.CTRLA = VREF_DAC0REFSEL_2V5_gc;       // Set VREF for DAC to 2.5V
}


int main(void)
{
   int ledState = 0;
   uint8_t fade = 0;
   uint16_t dac = 0;
   uint32_t end;
   
   initMCU();
   initGPIOs();
   initUARTs();
   initPWM();
   initMillisecondTimer();
   initDAC();
   
   sei();   // Enable interrupts
   
   printf("\nHello from the %s\n", "ATtiny1616");
   printResetReason();
   printFuses();
   printDeviceID();
   printSerialNumber();
   
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
            
         dac += 8;
         
         DAC0.DATA = dac >> 8;
         setRGBLed(ledState, fade);

         if (millis() >= end) {
            end = millis() + 500UL;
            PORTA.OUTTGL = LED;        // LED on PA1 toggle
            printf("millis() = %ld\n", millis());
         }
         
         Tick = 0;
      }
      
      if (UART0RxAvailable()) {
         const uint8_t ch = UART0RxByte();
         
         printf("UART0: %02x\n", ch);
         switch (ch) {
         case 'f':
         case 'F':
            printFuses();
            break;
         case 'i':
         case 'I':
            printDeviceID();
            break;
         case 'n':
         case 'N':
            printSerialNumber();
            break;
         case 'r':
         case 'R':
            printResetReason();
            break;
         }
      }
   }
}
