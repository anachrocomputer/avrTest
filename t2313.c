/* t2313 --- test code for ATtiny2313                      2019-11-26 */

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED    PB0  // Blinking LED on PB0
#define SQWAVE PB1  // 500Hz square wave on PB1

#define LED_R PB2   // Red LED on PB2
#define LED_G PB3   // Green LED on PB3
#define LED_B PB4   // Blue LED on PB4

#define BAUDRATE (9600)
#define BAUD_SETTING ((F_CPU / (BAUDRATE * 16UL)) - 1)

#define UART_RX_BUFFER_SIZE  (8)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (32)
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

uint8_t SavedMCUSR = 0;
volatile uint32_t Milliseconds = 0UL;
volatile uint8_t Tick = 0;


/* USART_RX_vect --- ISR for USART Receive Complete, used for Rx */

ISR(USART_RX_vect)
{
   const uint8_t tmphead = (U0Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
   const uint8_t ch = UDR;   // Read received byte from UART
   
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


/* USART_UDRE_vect --- ISR for USART Data Register Empty, used for Tx */

ISR(USART_UDRE_vect)
{
   if (U0Buf.tx.head != U0Buf.tx.tail) // Is there anything to send?
   {
      const uint8_t tmptail = (U0Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
      
      U0Buf.tx.tail = tmptail;

      UDR = U0Buf.tx.buf[tmptail];    // Transmit one byte
   }
   else
   {
      UCSRB &= ~(1 << UDRIE);  // Nothing left to send; disable Tx interrupt
   }
}


/* TIMER1_COMPA_vect --- ISR for Timer/Counter 1 overflow, used for 1ms ticker */

ISR(TIMER1_COMPA_vect)
{
   Milliseconds++;
   Tick = 1;
   PINB = (1 << SQWAVE);      // DEBUG: 500Hz on PB1 pin
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

   UCSRB |= (1 << UDRIE);   // Enable UART Tx interrupt
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


/* printResetReason --- print the cause of the chip's reset */

void printResetReason(void)
{
// TODO: make this work without calling 'printf()'
// printf("MCUSR = %02x\n", SavedMCUSR);
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
   DDRB |= (1 << LED) | (1 << LED_R) | (1 << LED_G) | (1 << LED_B) | (1 << SQWAVE);
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

   // Set baud rate
   UBRRH = (uint8_t)(BAUD_SETTING >> 8); 
   UBRRL = (uint8_t)(BAUD_SETTING);
   // Enable receive and transmit
   UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
   // Set frame format
   UCSRC = (1 << UCSZ0) | (1 << UCSZ1);  // Async 8N1

   stdout = &USART_stream;    // Allow use of 'printf' and similar functions
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

   puts("\nHello from the ATtiny2313");
   printResetReason();
   
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

      fputs("UU", stdout);

      _delay_ms(500);
      
//    OCR0A += 16;

      // Switch LED off
      PORTB &= ~(1 << LED);

      fputs("AB", stdout);
      
      _delay_ms(500);
      
      i = (i + 1) & 0x07;
   }
}
