/* t1284p --- test code for ATmega1284P                     2019-11-26 */

#ifndef F_CPU
#define F_CPU 20000000UL
#endif

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED    PB0  // Blinking LED on PB0
#define SQWAVE PB1  // 500Hz square wave on PB1

#define LED_R PD7   // Red LED on PD7
#define LED_G PB3   // Green LED on PB3
#define LED_B PB4   // Blue LED on PB4

#define BAUDRATE (9600)
#define BAUD_SETTING ((F_CPU / (BAUDRATE * 16UL)) - 1)

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
struct UART_BUFFER U1Buf;

uint8_t SavedMCUSR = 0;
volatile uint32_t Milliseconds = 0UL;
volatile uint8_t Tick = 0;


/* USART0_RX_vect --- ISR for USART0 Receive Complete, used for Rx */

ISR(USART0_RX_vect)
{
   const uint8_t tmphead = (U0Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
   const uint8_t ch = UDR0;  // Read received byte from UART
   
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


/* USART0_UDRE_vect --- ISR for USART0 Data Register Empty, used for Tx */

ISR(USART0_UDRE_vect)
{
   if (U0Buf.tx.head != U0Buf.tx.tail) // Is there anything to send?
   {
      const uint8_t tmptail = (U0Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
      
      U0Buf.tx.tail = tmptail;

      UDR0 = U0Buf.tx.buf[tmptail];    // Transmit one byte
   }
   else
   {
      UCSR0B &= ~(1 << UDRIE0);  // Nothing left to send; disable Tx interrupt
   }
}


/* USART1_RX_vect --- ISR for USART1 Receive Complete, used for Rx */

ISR(USART1_RX_vect)
{
   const uint8_t tmphead = (U1Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
   const uint8_t ch = UDR1;  // Read received byte from UART
   
   if (tmphead == U1Buf.rx.tail)   // Is receive buffer full?
   {
       // Buffer is full; discard new byte
   }
   else
   {
      U1Buf.rx.head = tmphead;
      U1Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
   }
}


/* USART1_UDRE_vect --- ISR for USART1 Data Register Empty, used for Tx */

ISR(USART1_UDRE_vect)
{
   if (U1Buf.tx.head != U1Buf.tx.tail) // Is there anything to send?
   {
      const uint8_t tmptail = (U1Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
      
      U1Buf.tx.tail = tmptail;

      UDR1 = U1Buf.tx.buf[tmptail];    // Transmit one byte
   }
   else
   {
      UCSR1B &= ~(1 << UDRIE1);  // Nothing left to send; disable Tx interrupt
   }
}


/* TIMER1_COMPA_vect --- ISR for Timer/Counter 1 overflow, used for 1ms ticker */

ISR(TIMER1_COMPA_vect)
{
   Milliseconds++;
   Tick = 1;
   PINB = (1 << SQWAVE);      // DEBUG: 500Hz on PB1 pin
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


/* t1ou0 --- transmit one character to UART0 by polling */

void t1ou0(const int ch)
{
   while ((UCSR0A & (1 << UDRE0)) == 0)
      ;
      
   UDR0 = ch;
}


/* t1ou1 --- transmit one character to UART1 by polling */

void t1ou1(const int ch)
{
   while ((UCSR1A & (1 << UDRE1)) == 0)
      ;
      
   UDR1 = ch;
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

   UCSR0B |= (1 << UDRIE0);   // Enable UART0 Tx interrupt
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


/* UART0RxAvailable --- return true if a byte is available in the UART0 circular buffer */

int UART0RxAvailable(void)
{
   return (U0Buf.rx.head != U0Buf.rx.tail);
}


/* UART1RxByte --- read one character from UART1 via the circular buffer */

uint8_t UART1RxByte(void)
{
   const uint8_t tmptail = (U1Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
   
   while (U1Buf.rx.head == U1Buf.rx.tail)  // Wait, if buffer is empty
       ;
   
   U1Buf.rx.tail = tmptail;
   
   return (U1Buf.rx.buf[tmptail]);
}


/* UART1TxByte --- send one character to UART1 via the circular buffer */

void UART1TxByte(const uint8_t data)
{
   const uint8_t tmphead = (U1Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
   
   while (tmphead == U1Buf.tx.tail)   // Wait, if buffer is full
       ;

   U1Buf.tx.buf[tmphead] = data;
   U1Buf.tx.head = tmphead;

   UCSR1B |= (1 << UDRIE1);   // Enable UART1 Tx interrupt
}


/* UART1RxAvailable --- return true if a byte is available in the UART1 circular buffer */

int UART1RxAvailable(void)
{
   return (U1Buf.rx.head != U1Buf.rx.tail);
}


/* setRGBLed --- control RGB LED connected to PORT B */

void setRGBLed(const int state, const uint8_t fade)
{
   switch (state) {
   case 0:                    // Red fading up, blue on
      OCR2A = fade;
      OCR0A = 0;
      OCR0B = 255;
      PORTD |= (1 << LED_R);
      PORTB &= ~(1 << LED_G);
      PORTB &= ~(1 << LED_B);
      break;
   case 1:                    // Red on, blue fading down
      OCR2A = 255;
      OCR0A = 0;
      OCR0B = 255 - fade;
      PORTD |= (1 << LED_R);
      PORTB |= (1 << LED_G);
      PORTB &= ~(1 << LED_B);
      break;
   case 2:                    // Red on, green fading up
      OCR2A = 255;
      OCR0A = fade;
      OCR0B = 0;
      PORTD &= ~(1 << LED_R);
      PORTB |= (1 << LED_G);
      PORTB &= ~(1 << LED_B);
      break;
   case 3:                    // Red fading down, green on
      OCR2A = 255 - fade;
      OCR0A = 255;
      OCR0B = 0;
      PORTD &= ~(1 << LED_R);
      PORTB |= (1 << LED_G);
      PORTB |= (1 << LED_B);
      break;
   case 4:                    // Green on, blue fading up
      OCR2A = 0;
      OCR0A = 255;
      OCR0B = fade;
      PORTD &= ~(1 << LED_R);
      PORTB &= ~(1 << LED_G);
      PORTB |= (1 << LED_B);
      break;
   case 5:                    // Green fading down, blue on
      OCR2A = 0;
      OCR0A = 255 - fade;
      OCR0B = 255;
      PORTD |= (1 << LED_R);
      PORTB &= ~(1 << LED_G);
      PORTB |= (1 << LED_B);
      break;
   }
}


/* printResetReason --- print the cause of the chip's reset */

void printResetReason(void)
{
   printf("MCUSR = %02x\n", SavedMCUSR);
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
   DDRB |= (1 << LED) | (1 << LED_G) | (1 << LED_B) | (1 << SQWAVE);
   DDRD |= (1 << PD6) | (1 << LED_R);
   PORTB = 0;  // All LEDs off
}


/* initUARTs --- set up UART(s) and buffers, and connect to 'stdout' */

static void initUARTs(void)
{
   // Set up UART0 and associated circular buffers
   U0Buf.tx.head = 0;
   U0Buf.tx.tail = 0;
   U0Buf.rx.head = 0;
   U0Buf.rx.tail = 0;

   UBRR0H = (uint8_t)(BAUD_SETTING >> 8); 
   UBRR0L = (uint8_t)(BAUD_SETTING);
   // Enable receive and transmit
   UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
   // Set frame format
   UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);  // Async 8N1

   // Set up UART1 and associated circular buffers
   U1Buf.tx.head = 0;
   U1Buf.tx.tail = 0;
   U1Buf.rx.head = 0;
   U1Buf.rx.tail = 0;

   UBRR1H = (uint8_t)(BAUD_SETTING >> 8); 
   UBRR1L = (uint8_t)(BAUD_SETTING);
   // Enable receive and transmit
   UCSR1B = (1 << RXCIE1) | (1 << RXEN1) | (1 << TXEN1);
   // Set frame format
   UCSR1C = (1 << UCSZ10) | (1 << UCSZ11);  // Async 8N1

   stdout = &USART_stream;    // Allow use of 'printf' and similar functions
}


/* initPWM --- set up PWM channels */

static void initPWM(void)
{
   // Config Timer 0 for PWM
   TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
   TCCR0B = (1 << CS01);   // Clock source = CLK/8, start PWM
   OCR0A = 0x80;
   OCR0B = 0x80;

   // Config Timer 2 for PWM
   TCCR2A = (1 << COM2A1) | (1 << COM2B1) | (1 << WGM20);
   TCCR2B = (1 << CS21);   // Clock source = CLK/8, start PWM
   OCR2A = 0x80;
   OCR2B = 0x80;
}


/* initMillisecondTimer --- set up a timer to interrupt every millisecond */

static void initMillisecondTimer(void)
{
   // Set up Timer/Counter 1 for regular 1ms interrupt
   TCCR1A = 0;             // WGM11 and WGM10 are set to 0 for CTC mode
   TCCR1B = (1 << WGM12) | (1 << CS10);   // WGM13 set to 0 and WGM12 set to 1 for CTC mode
                                          // CS10 set to 1 for divide-by-1 prescaler
   OCR1A = 19999;                // 20000 counts gives 1ms
   TCNT1 = 0;
   TIMSK1 = (1 << OCIE1A);       // Enable interrupts
}


int main(void)
{
   int ledState = 0;
   uint8_t fade = 0;
   uint32_t end;

   initMCU();
   initGPIOs();
   initUARTs();
   initPWM();
   initMillisecondTimer();

   sei();   // Enable interrupts
   
   UART1TxByte('\r');
   UART1TxByte('\n');

   printf("\nHello from the %s\n", "ATmega1284P");
   printResetReason();

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

         if (millis() >= end) {
            end = millis() + 500UL;

            PINB = (1 << LED);         // LED on PA1 toggle

            UART1TxByte('U');
            UART1TxByte('1');
            UART1TxByte(' ');
            UART1TxByte('1');
            UART1TxByte('2');
            UART1TxByte('8');
            UART1TxByte('4');
            UART1TxByte('P');
            UART1TxByte(' ');

            printf("millis() = %ld\n", millis());
         }
         
         Tick = 0;
      }

      if (UART0RxAvailable()) {
         const uint8_t ch = UART0RxByte();
         
         printf("UART0: %02x\n", ch);
         switch (ch) {
         case 'r':
         case 'R':
            printResetReason();
            break;
         }
      }
   }
}
