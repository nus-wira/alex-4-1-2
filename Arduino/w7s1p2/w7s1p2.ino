#include <avr/io.h>
#include <avr/interrupt.h>

/*#ifndef F_CPU
#define F_CPU   16000000UL
#endif
*/
#include <util/delay.h>

// Masks for pins 12 and 11
#define PIN12MASK   0b00010000
#define PIN11MASK   0b00001000

// UDRIE mask. Use this to enable/disable
// the UDRE interrupt
#define UDRIEMASK   0b00100000

static volatile char flashWhich=1;
static volatile char buttonVal=1;

char dataRecv, dataSend;


ISR(USART_RX_vect)
{
  // Write received data to dataRecv
  dataRecv = UDR0;
}

ISR(USART_UDRE_vect)
{
  // Write dataSend to UDR0
  UDR0 = dataSend;
  // Disable UDRE interrupt
  // UDRIE0 is bit 5
  UCSR0B &= !UDRIEMASK;
}
/**/

void sendData(const char data)
{
  // Copy data to be sent to dataSend
  dataSend = data;

  // Enable UDRE interrupt below
  // UDRIE0 is bit 5
  UCSR0B |= UDRIEMASK;
}

char recvData()
{
  
  return dataRecv - '0';
}

void setupSerial()
{
  // Set up the 115200 8N1 when using
  // Serial Monitor to test
 
  // Change to 115200 7E1 when
  // communicating between Arduinos.

  // b = round(16000000 / (16*9600)) - 1 = 103
  // 103 < 255, load directly into UBRR0L,
  // while setting UBRR0H to 0

  UBRR0L = 103;
  UBRR0H = 0;

  /*
  Asynchronous -> USCR0C[7:6] 00
  No parity -> USCR0C[5:4] 00
  1 stop bit -> USCR0C[3] 0
  8 bits -> UCSZ0[2:0] 011
  UCSZ0 [1:0] corresponds to UCSR0C[2:1]
  -> USCR0C[2:1] 11
  bit 0 (UCPOL0) is always 0
  */
  
  USCR0C = 0b00000110;
  /**/
  /*
  For Arduino,
  Even parity -> USCR0C[5:4] 10
  7 bits -> UCSZ0[2:0] 010
  */
  /*
  USCR0C = 0b00100100;
  /**/

  /* 
  Zero everything in UCSR0A, espy U2X0 and MPCM0 
  to ensure we are not in double speed mode and 
  are also not in multiprocessor mode, 
  which will discard frames without addresses 
  */
  UCSR0A = 0;
  
}

void startSerial()
{
  // Start the serial port.
  // Enable RXC interrupt, but NOT UDRIE
  // Remember to enable the receiver
  // and transmitter
  /*
  Once we set RXEN0 and TXEN0 (bits 4 and 3),
  we will start the UART.
  Hence, place setup for UCSR0B in separate
  startSerial function.

  Using UDRE and RXC interrupts, so set RXCIE0 (bit 7)
  and UDRIE0 (bit 5) to 1.
  Disable TXCIE0 at bit 6.
  -> [7:5] 101
  Enable receiver and transmitter resp. -> [4:3] 11
  Bit 2 corresponds to UCSZ02 -> [2] 0 (See USCR0C)
  RXB80 and TXB80 are always 00 since not using 9-bit
  -> [1:0] 00

  We will use UDR0 empty interrupt for transmitting,
  but for now set UDRIE0 to 0.
  */
  UCSR0B = 0b10011000;
}

// Enable external interrupts 0 and 1
void setupEINT()
{
  // Configure INT0 and INT1 for rising edge triggered.
  // Remember to enable INT0 and INT1 interrupts.
  // Rising Edge is 11 for both
  EICRA = 0b00001111;
  EIMSK = 0b00000011;
}


// ISRs for external interrupts
ISR(INT0_vect)
{
  buttonVal=1;  
  sendData('0'+buttonVal);
}

ISR(INT1_vect)
{
  buttonVal=2;
  sendData('0'+buttonVal);
}

// Red is on pin 12
void flashRed()
{
    PORTB |= PIN12MASK;
    delay(100);
    PORTB &= ~PIN12MASK;
    delay(500);
}

// Green is on pin 11
void flashGreen()
{
    PORTB |= PIN11MASK;
    delay(100);
    PORTB &= ~PIN11MASK;
    delay(500);
}

void setup() {

  cli();
  // put your setup code here, to run once:

  DDRB |= (PIN11MASK | PIN12MASK);
  setupEINT();
  setupSerial();
  startSerial();
  sei();
}

void loop() {
  // put your main code here, to run repeatedly:

  flashWhich = recvData();
   
  if(flashWhich == 1)
    flashRed();
  else
    flashGreen();
}
