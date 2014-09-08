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




ISR(INT0_vect) {
    uint16_t timer = 0;
    while(bit_is_clear(PIND, PD2)) { // button hold down
        timer++;
        _delay_ms(1);
    }
    if(timer > BTN_DEBOUCE) { // software debouncing button
        if(timer < 500UL) {//unsigned long
            //single click
            //printStringFromPROGMEM(serial_msg_1); // doesn't work
        } else {
            //button hold
            //printStringFromPROGMEM(serial_msg_2);
        }
    }
}

//all this part needs refactoring

const char serial_msg_1[] PROGMEM = "single click\r\n";
const char serial_msg_2[] PROGMEM = "hold down\r\n";
const char serial_msg_3[] PROGMEM = "Read from EEPROM";
const char serial_msg_4[] PROGMEM = "Write to EEPROM";
const char serial_msg_5[] PROGMEM = "CMD";
const char serial_br[] PROGMEM = "\r\n";

struct SerialCommand *getSerialPack(void); // metti in header

struct SerialCommand{
	uint8_t len;
	uint16_t cmd;
    uint8_t *param;
};


ISR(USART_RX_vect) {
	cli();
	struct SerialCommand *command = getSerialPack();

	switch(command -> cmd) {
		case READ_FROM_EEPROM:
			printStringFromPROGMEM(serial_br);
			printStringFromPROGMEM(serial_msg_3);
			printStringFromPROGMEM(serial_br);
			printString("Address:");
			printHexByte(command -> param[0]);
			break; 
		case WRITE_TO_EEPROM:
			printStringFromPROGMEM(serial_msg_4);
			break;

	}
	// printHexByte((command -> cmd) >> 8);
 //    printHexByte(command -> cmd);
 //    printHexByte(command -> len);
	sei();
}

struct SerialCommand *getSerialPack() { // format: 2bytes header, 1byte len, 2bytes command, 1..n bytes params

    struct SerialCommand *sc= malloc(sizeof(struct SerialCommand));
    

    //uint8_t checksum;
  
    if(receiveByte() != HEADER_BYTE_1) {
      return;    
    }
    if(receiveByte() != HEADER_BYTE_2) {
      return;
    }
    //init packet
    sc ->len = receiveByte();
    sc -> cmd = receiveByte() << 8;
    sc -> cmd |= receiveByte();

    uint8_t len = sc -> len;
    
    if(len > 2) { // parameters available
        for(uint8_t i = 0; i<(len-2); i++) {
           sc -> param[i] = receiveByte(); 
        }
    }
    
    //WAIT_FOR_SERIAL_DATA;
    //checksum = Serial.read();
    //if(checksum == (len+(cmd >> 8)+(cmd)+)) 
    
    // printHexByte(HEADER_BYTE_1);
    // printHexByte(HEADER_BYTE_2);
    // printHexByte(len);
    // printHexByte((sc -> cmd) >> 8);
    // printHexByte(sc -> cmd);
    // if(len > 2) {
    //   for(uint8_t i = 0; i<(len-2); i++) {
    //     printHexByte(sc->param[i]);
    //   }  
    // }

    return sc;
    
    
        
}

void initInterrupt0(void) {
    GIMSK |= (1 << INT0); // general interrupt mask register -> enable interrupt on INT0
    MCUCR &= ~(1 << ISC00); // trigger interrupt on falling edge
    MCUCR |= (1 << ISC01); // trigger interrupt on falling edge
    sei(); //  set interrupt enable bit
}

//READ

//PC send command RD
//MCU respond OK
//PC send EEPROM address from witch start reading and number of bytes
//MCU respond with readed bytes

//WRITE

//PC send command WR
//MCU respond OK
//PC send address number in witch start write and number of bytes
//MCU writes data and at the end respond WR-OK to computer

int main(void) {


  // -------- Inits --------- //

  // button
  DDRD &= ~_BV(PD2);
  PORTD |= _BV(PD2); //turn pull-up on


  //need this to make eeprom work
  //DDRB = ~_BV(PB5); // 0x11011111 -> MOSI PB5 (Serial Data in) -- Connected to EEPROM Serial Output


  initUSART();
  initSPI();
  initEEPROM();
  initInterrupt0();


  printString("\r\nWelcome to Fled!\r\n");                          /* to test */


  EEPROM_clear();


  // printString("Write to EEPROM.\r\n");

  // EEPROM_writeByte(0,~0b11110000);
  // EEPROM_writeByte(1,~0b10000000);

  //EEPROM_writePage(0x0000, data);


  printBinaryByte(EEPROM_readByte(0));

  //_delay_ms(3000);

  EEPROM_readIntoLeds(0);




  // Shift Register
  PORTD &= ~_BV(PD5);
  spiTransfer(~0b11111111);
  spiTransfer(~0b11111111);
  PORTD |= _BV(PD5);

  // printString("Read from EEPROM: \r\n");
  // // for(uint16_t i= 0; i< 512; i++) {
  // //     printHexByte(i);
  // //     printString(" > ");
  // //     ;
  // //     printString("\r\n");
  // // }

  // uint8_t *buf = EEPROM_readPage(0x0000);
  // for(uint8_t i = 0; i < 32; i++) {
  //   printBinaryByte(buf[i]);
  //   printString("\r\n");
  // }

  // printString("\r\n");





  // ------ Event loop ------ //
  while (1) {
    _delay_ms(1000);
    PORTB ^= (1 << PB0);
  }                                                  /* End event loop */
  return (0);
}
