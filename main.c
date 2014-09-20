/*
A simple test of serial-port functionality.
Takes in a character at a time and sends it right back out,
 displaying the ASCII value on the LEDs.
*/

// ------- Preamble -------- //
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include "USART.h"
#include "SPI.h"
#include "SPI_EEPROM.h"


#define BTN_DEBOUCE 20 //ms
#define DATA_MODE 0x01
#define CMD_MODE 0x02
#define WRITE 0x01
#define READ 0x02

#define READ_FROM_EEPROM 0x0001
#define WRITE_TO_EEPROM 0x0002

#define HEADER_BYTE_1 0xFF
#define HEADER_BYTE_2 0x55



//all this part needs refactoring

const char serial_msg_1[] PROGMEM = "single click\r\n";
const char serial_msg_2[] PROGMEM = "hold down\r\n";
const char serial_msg_3[] PROGMEM = "Read from EEPROM";
const char serial_msg_4[] PROGMEM = "Write to EEPROM";
const char serial_msg_5[] PROGMEM = "CMD";
const char serial_msg_6[] PROGMEM = "OK";
const char serial_br[] PROGMEM = "\r\n";

struct SerialCommand *getSerialPack(void); // metti in header

struct SerialCommand{
	uint8_t len;
	uint16_t cmd;
    uint8_t *param;
};

struct SerialCommand *getSerialPack() { // format: 2bytes header, 1byte len, 2bytes command, 1..n bytes params


	//@TODO add checksum
    //uint8_t checksum;
  
    if(receiveByte() != HEADER_BYTE_1) {
      return;    
    }
    if(receiveByte() != HEADER_BYTE_2) {
      return;
    }

    struct SerialCommand *sc= NULL;

    sc = (struct SerialCommand *) malloc(sizeof(struct SerialCommand));
    
    if(!sc) {
    	printString("NO MEM");
    	exit(1);
    }

    //init packet
    sc ->len = receiveByte();
    sc -> cmd = receiveByte() << 8;
    sc -> cmd |= receiveByte();

    
    if(sc -> len > 2) { // parameters available
    	*sc->param = NULL;
    	sc->param = (uint8_t *) malloc((sc->len)-2);
    	if(!sc->param) {
    		printString("NO MEM");
    		exit(1);
    	}
        for(uint8_t i = 0; i<(sc->len)-2; i++) {
           sc -> param[i] = receiveByte(); 
        }
    }

    return sc;   
        
}


ISR(USART_RX_vect) {
	cli();
	struct SerialCommand *command = NULL;
	command = getSerialPack();

	uint8_t eeprom_address;
	uint8_t len;

	switch(command -> cmd) {
		case READ_FROM_EEPROM:

			eeprom_address = (command -> param[0]);
			len = (command -> param[1]);

			#ifdef DEBUG
				printStringFromPROGMEM(serial_br);
				printStringFromPROGMEM(serial_msg_3);
				printStringFromPROGMEM(serial_br);
				printString("Address:");
				printHexByte(eeprom_address);
				printStringFromPROGMEM(serial_br);
				printString("Bytes:");
				printHexByte(len);
				printStringFromPROGMEM(serial_br);
			#endif

			for(uint8_t i = 0; i< len; i++) {
				uint8_t read = EEPROM_readByte(eeprom_address);
				#ifdef DEBUG
					printBinaryByte(read);
				#else
					transmitByte(read);
				#endif

				EEPROM_readIntoLeds(eeprom_address);
				eeprom_address += 1;
			}
			

			break; 
		case WRITE_TO_EEPROM:

			#ifdef DEBUG
				printStringFromPROGMEM(serial_br);
				printStringFromPROGMEM(serial_msg_4);
				printStringFromPROGMEM(serial_br);
				printString("Address:");
				printHexByte(eeprom_address);
				printStringFromPROGMEM(serial_br);
				printString("Data:");
			#endif

			eeprom_address = (command -> param[0]);

			for(uint8_t i = 1; i <= (command->len)-3; i++ ) {
				#ifdef DEBUG
					printHexByte(command -> param[i]);
				#endif	
				EEPROM_writeByte(eeprom_address, command -> param[i]);
				eeprom_address += 1;
			}
			printStringFromPROGMEM(serial_msg_6); // OK


			break;

	}
	//free allocate memory here
	free(command -> param);
	free(command);

	sei();
}

ISR(INT0_vect) {
    uint16_t timer = 0;
    while(bit_is_clear(PIND, PD2)) { // button hold down
        timer++;
        _delay_ms(1);
    }
    if(timer > BTN_DEBOUCE) { // software debouncing button
        if(timer < 500UL) {//unsigned long
            //single click
            printStringFromPROGMEM(serial_msg_1); // doesn't work
        } else {
            //button hold
            printStringFromPROGMEM(serial_msg_2);
        }
    }
}



void initInterrupt0(void) {
    GIMSK |= (1 << INT0); // general interrupt mask register -> enable interrupt on INT0
    MCUCR &= ~(1 << ISC00); // trigger interrupt on falling edge
    MCUCR |= (1 << ISC01); // trigger interrupt on falling edge
    sei(); //  set interrupt enable bit
}


int main(void) {


  // -------- Inits --------- //

  DDRB |= _BV(PB0); 

  // button
  DDRD &= ~_BV(PD2);
  PORTD |= _BV(PD2); //turn pull-up on


  //need this to make eeprom work
  //DDRB = ~_BV(PB5); // 0x11011111 -> MOSI PB5 (Serial Data in) -- Connected to EEPROM Serial Output


  initUSART();
  initSPI();
  initEEPROM();
  initInterrupt0();


  printString("\r\nWelcome to Fled!\r\n");


  EEPROM_clear();


  // printString("Write to EEPROM.\r\n");

  EEPROM_writeByte(0,~0b11110000);
  EEPROM_writeByte(1,~0b10000000);
  EEPROM_writeByte(2,~0b10101010);
  EEPROM_writeByte(3,~0b00010001);

  //EEPROM_writePage(0x0000, data);


  printBinaryByte(EEPROM_readByte(0));

  //_delay_ms(3000);

  //EEPROM_readIntoLeds(0);







  // ------ Event loop ------ //
  while (1) {
    _delay_ms(1000);
    PORTB ^= (1 << PB0);
  }                                                  /* End event loop */
  return (0);
}
