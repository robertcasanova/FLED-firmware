#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "USART.h"

#define BAUD 115200
#define SENSOR_PWR PD6
#define SENSOR PD3




uint8_t counter = 0;

ISR(TIMER1_OVF_vect) {
	printString("\r\n");
	printString("OVERFLOW");
	//should go in standby
}

ISR(TIMER0_COMPA_vect) {
	
	printWord(TCNT1);
	printString("\r\n");	
	
}

//call every loop
ISR(INT1_vect) { 
	counter = 0;
	OCR0A = TCNT1 >> 7; // timercounter1 / 128

	//OCR0A = TCNT1 >> 1;// /64 divided by 2 = /128
	TCNT1 = 0;
	printString("\r\n\r\n");
	//counter = 0;
	//printString("\r\n\r\n");	
	// printString("\r\n");
	// printWord(1000 / (TCNT1 * 0.064));
	// printString("FPS");
	//OCR0A = TCNT1 >> 7; // is same of /128
	//TCNT1 = 0;
	//TCNT0 = 0;
	
}

static void initInterruptHallSensor(void) {
	MCUCR |= _BV(ISC11);// trigger interrupt on falling edge
  	GIMSK |= _BV(INT1); 
}

void initTimer0(void) {
	TCCR0A |= (1 << WGM01); // set operation mode to CTC Clear timer on Compare
	TCCR0B |= (1 << CS01) | (1<< CS00); // prescaler 1000000 / 64 = 15,625 millisec x tick
	
	TIMSK |= _BV(OCIE0A);

}


void initTimer1(void) {
	//normal with prescaler 1000000 / 64 = 15,625 millisec x tick
	TCCR1B |= (1 << CS11) | (1 << CS10);
	TIMSK |= _BV(TOIE1);
	

}

int main(void) {

	// button & hall sensor
  	DDRD &= ~_BV(SENSOR);
  	DDRD |= _BV(SENSOR_PWR);

  	PORTD |= _BV(SENSOR) | _BV(SENSOR_PWR); //turn pull-up on

  	sei();

	initUSART();
	initTimer0();
	initTimer1();
	initInterruptHallSensor();


	TCNT0 = 0;
	TCNT1 = 0;

	printString("TEST");

	while(1) {}

	return 0;
}