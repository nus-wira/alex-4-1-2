#include "Arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define RF 6   // Left forward pin
#define RR 5   // Left reverse pin
#define LF 10  // Right forward pin
#define LR 9  // Right reverse pin
void setup() {
  // put your setup code here, to run once:
  cli();
  DDRD |= 0b01100000;
  DDRB |= 0b00000110;
  TCNT1 = 0;
  TCNT0 = 0;
  TCCR0A = 0b10100001;
  // Both on with mode 1
  TCCR1A = 0b10100001;

  // Both on with mode 3
//  TCCR1A = 0b10100011;
  // Both on with mode 11
//  TCCR1A = 0b10100011;

  //Only A on with mode 1
//  TCCR1A = 0b10000001;
//  TIMSK0 |= 0b110; // Enable Int for Output Compare Match
  OCR1AH = 0;
  OCR1AL = 0;
  OCR1BH = 0;
  OCR1BL = 0;
  //Both on with mode 11
//  TCCR1B = 0b00010001;
  //Both on with mode 3
//  TCCR1B = 0b00000001;
  //Both on with mode 1
//  TCCR1B = 0b00000001;
  // Only A on with mode 1
  TCCR1B = 0b00000001;
  
  OCR0A = 0;
  OCR0B = 0;
  TCCR0B = 0b00000011;
  sei();
}
ISR(TIMER0_COMPA_vect) {
}
ISR(TIMER0_COMPB_vect) {
}

void loop() {
  // put your main code here, to run repeatedly:
  int i = 1000000;
//  OCR0A = 128;
//  OCR0B = 128;
  OCR1AH = 0;
  OCR1AL = 128;
//  OCR1BL = 128;
//  while (i--);
//  OCR0A = 0;
////  OCR1BL = 0;
//  i = 1000000;
//  while(i--);
//  OCR0B = 150;
////  OCR1AL = 150;
//  i = 1000000;
//  while(i--);
//  OCR0B = 0;
//  OCR1AL = 0;
}
//
