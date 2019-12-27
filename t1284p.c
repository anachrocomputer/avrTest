#ifndef F_CPU
#define F_CPU 20000000UL
#endif

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LED PB0

#define LED_R PB2
#define LED_G PB3
#define LED_B PB4

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

uint8_t SavedMCUSR = 0;


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

void t1ou0(const int ch)
{
   while ((UCSR0A & (1 << UDRE0)) == 0)
      ;
      
   UDR0 = ch;
}

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
      t1ou0('\r');

   t1ou0(c);

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
   printf("MCUSR = %02x\n", SavedMCUSR);
}


int main(void)
{
   int i = 0;
   
   SavedMCUSR = MCUSR;
   MCUSR = 0;
   
   // Set up output pins
   DDRB |= (1 << LED) | (1 << LED_R) | (1 << LED_G) | (1 << LED_B);
   PORTB = 0;  // ALl LEDs off
   
   // Set baud rate on UART0
   UBRR0H = (uint8_t)(BAUD_SETTING >> 8); 
   UBRR0L = (uint8_t)(BAUD_SETTING);
   // Enable receive and transmit
   UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
   // Set frame format
   UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);  // Async 8N1

   // Set baud rate on UART1
   UBRR1H = (uint8_t)(BAUD_SETTING >> 8); 
   UBRR1L = (uint8_t)(BAUD_SETTING);
   // Enable receive and transmit
   UCSR1B = (1 << RXEN1) | (1 << TXEN1);
   // Set frame format
   UCSR1C = (1 << UCSZ10) | (1 << UCSZ11);  // Async 8N1

   stdout = &USART_stream;    // Allow use of 'printf' and similar functions

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

   sei();   // Enable interrupts
   
   t1ou1('\r');
   t1ou1('\n');

   printf("\nHello from the %s\n", "ATmega1284P");
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

      printf("LED ON\n");

      t1ou1('U');
      t1ou1('U');

      _delay_ms(500);
      
//    OCR0A += 16;

      // Switch LED off
      PORTB &= ~(1 << LED);

      printf("LED OFF\n");
      
      t1ou1('C');
      t1ou1('D');
      
      _delay_ms(500);
      
      i = (i + 1) & 0x07;
      
      if (UART0RxAvailable()) {
         const char ch = UART0RxByte();
         
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
