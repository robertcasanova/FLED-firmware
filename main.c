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

 
// #define DEBUG true

#define BTN_DEBOUCE 20 //ms
#define HALL_DEBOUNCE_THRESH  4

#define DATA_MODE 0x01
#define CMD_MODE 0x02
#define WRITE 0x01
#define READ 0x02

#define READ_FROM_EEPROM 0x0001
#define WRITE_TO_EEPROM 0x0002

#define HEADER_BYTE_1 0xFF
#define HEADER_BYTE_2 0x55



#define SENSOR_PWR PD6
#define SENSOR PD3
#define BUTTON PD2
#define LATCH PD5

//global variables used inside ISR
volatile uint8_t hall_debounce;
volatile uint16_t sensor_timer;


//all this part needs refactoring

const char serial_msg_1[] PROGMEM = "single click\r\n";
const char serial_msg_2[] PROGMEM = "hold down\r\n";
const char serial_msg_3[] PROGMEM = "Read from EEPROM";
const char serial_msg_4[] PROGMEM = "Write to EEPROM";
const char serial_msg_5[] PROGMEM = "CMD";
const char serial_msg_6[] PROGMEM = "OK";
const char serial_msg_7[] PROGMEM = "Welcome to Fled!";
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

    printHexByte(receiveByte());
  
    if(receiveByte() != HEADER_BYTE_1) {

      return;    
    }
    if(receiveByte() != HEADER_BYTE_2) {
      return;
    }



    struct SerialCommand *sc= NULL;

    //allocate memory for struct
    sc = (struct SerialCommand *) malloc(sizeof(struct SerialCommand));
    


    if(!sc) {
    	exit(1);
    }

    //init packet
    sc -> len = receiveByte();
    sc -> cmd = receiveByte() << 8;
    sc -> cmd |= receiveByte();

    

    
    if(sc -> len > 2) { // parameters available
    	*sc->param = NULL;
    	sc->param = (uint8_t *) malloc((sc->len)-2);
    	if(!sc->param) {
    		exit(1);
    	}
        for(uint8_t i = 0; i<(sc->len)-2; i++) {
           sc -> param[i] = receiveByte(); 
        }
    }

    return sc;   
        
}

//receive commans from computer
ISR(USART_RX_vect) {
	cli();
	printStringFromPROGMEM(serial_br);


	struct SerialCommand *command = NULL;
	command = getSerialPack();

	


	uint8_t eeprom_address;
	uint8_t len;

	switch(command -> cmd) {
		case READ_FROM_EEPROM:

			eeprom_address = (command -> param[0]);
			len = (command -> param[1]);

				// printStringFromPROGMEM(serial_br);
				// printStringFromPROGMEM(serial_msg_3);
				// printStringFromPROGMEM(serial_br);
				// printString("Address:");
				// printHexByte(eeprom_address);
				// printStringFromPROGMEM(serial_br);
				// printString("Bytes:");
				// printHexByte(len);
				// printStringFromPROGMEM(serial_br);

			for(uint8_t i = 0; i< len; i++) {
				uint8_t read = EEPROM_readByte(eeprom_address);
					printBinaryByte(read);

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
      #ifdef DEBUG
			  printStringFromPROGMEM(serial_msg_6); // OK
      #endif

			break;

	}
	//free allocate memory here
	free(command -> param);
	free(command);

	sei();
}

//call every 64ms
ISR(TIMER0_OVF_vect) { 
  //increment sensor_timer and hall debounce
  if(hall_debounce != 0xFF) {
    hall_debounce++;
  }
  if(sensor_timer != 0xFFFF) {
    sensor_timer++;
  }
}

//called every step of the circle, 128 steps per circle
ISR(TIMER1_COMPA_vect) {
  //read data into leds
  #ifdef DEBUG
  printWord((sensor_timer << 8) | TCNT0);
  printStringFromPROGMEM(serial_br);
  #endif 
}

//button interrupt
ISR(INT0_vect) {

    uint16_t timer = 0;
    while(bit_is_clear(PIND, BUTTON)) { // button hold down, 0 because we are using an internal pull-up
        timer++;
        _delay_ms(1);
    }
    if(timer > BTN_DEBOUCE) { // software debouncing button
        if(timer < 500UL) {//unsigned long
            //single click
            //@TODO reset the watchdog timer, will reset the device
            #ifdef DEBUG
              printStringFromPROGMEM(serial_msg_1); 
            #endif
        } else {
          sensor_timer = 0xFFFF;
            //button hold
          #ifdef DEBUG
            printStringFromPROGMEM(serial_msg_2);
          #endif
        }
    }
}

//hall sensor interrupt
//ISR(INT1_vect) {
static void hallSensorFakeInterrupt(void){
  if(hall_debounce > HALL_DEBOUNCE_THRESH) { // is passed at least 64ms*4 since last loop
      TCNT1 = 0; 
      if((sensor_timer < 0xFF) && (sensor_timer > 0x3)) {
        OCR1A = (sensor_timer << 8) | TCNT0; // store the count of timer0 since last loop
        TCNT0 = 0; // reset timer0
        // get current EEPROM address
        TCCR1B |= _BV(CS10);// start timer1 without prescaler at 100000hz
        TIMSK |= _BV(OCIE1A); // reset counter when reaches OCR1A value and trigger interrupt
      } else {
        TCCR1B &= ~_BV(CS10);//stop timer
      }
      sensor_timer = 0;

  }
  hall_debounce = 0;
}




static void initInterruptButton(void) {
  MCUCR |= _BV(ISC01); // trigger interrupt on falling edge
  GIMSK |= _BV(INT0); // general interrupt mask register -> enable interrupt on INT0   
}

static void initInterruptHallSensor(void) {
	MCUCR |= _BV(ISC11);// trigger interrupt on falling edge
  GIMSK |= _BV(INT1); 
}

static void initTimer1(void) {
	//Timer Counter 1 Control Register B, CS12 0 CS11 1 CS10 1 ---> F_CPU / 64 (clock source)
  	TCCR1A = 0;
	TCCR1B = _BV(WGM12); // waveform generation mode, CTC mode, compare with OCR1A
}

static void initTimer0(void) {
	//Timer Counter 0 Control  Register A, 0 = Normal Mode (compare disconnected)
	TCCR0A = 0; 
	//TImer Counter 0 Control Register B, clock = F_CPU / 265 = 1000000 / 256 = ~0,25 ms x tick
	TCCR0B = _BV(CS02);
	//Timer Counter interrupt Mask, enable Timercounter0verflowInterruptEnable0
	// 8bit = max 256, 256*0,25 = 64ms (overflow every)
	TIMSK = _BV(TOIE0);
}


int main(void) {

  // -------- Inits --------- //

  // button & hall sensor
  DDRD &= ~_BV(BUTTON) & ~_BV(SENSOR);
  DDRD |= _BV(SENSOR_PWR) | _BV(LATCH);

  PORTD |= _BV(BUTTON) | _BV(SENSOR) | _BV(SENSOR_PWR); //turn pull-up on
  PORTD &= ~_BV(LATCH); // deselect latch


  initUSART();
  initSPI();
  initEEPROM();
  initInterruptButton();
  initInterruptHallSensor();
  initTimer0();
  initTimer1();
  sei();

  hall_debounce = 0;
  sensor_timer = 0;

  printString("Start");
  
  #ifdef DEBUG
  printStringFromPROGMEM(serial_br);
  printStringFromPROGMEM(serial_msg_7);
  printStringFromPROGMEM(serial_br);
  #endif

  // printString("EEPROM Clear.\r\n");
  //EEPROM_clear();



  //EEPROM_writeByte(0,~0b11001100);
  //EEPROM_writeByte(1,~0b11110000);

  //EEPROM_writePage(0x0000, data);


  //printBinaryByte(EEPROM_readByte(0));


  //_delay_ms(3000);

  //EEPROM_readIntoLeds(0);

  

  while(1){
      hallSensorFakeInterrupt();
      _delay_ms(1000); // 1400rpm motor = 43ms x loop
  };
                                                  /* End event loop */
  return (0);
}
