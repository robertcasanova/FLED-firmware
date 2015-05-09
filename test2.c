#include <avr/io.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "avr_mcu_section.h"

AVR_MCU(1000000, "atmega88");

const struct avr_mmcu_vcd_trace_t _mytrace[]  _MMCU_ = {
    { AVR_MCU_VCD_SYMBOL("DDRB"), .what = (void*)&DDRB, },
    { AVR_MCU_VCD_SYMBOL("PORTB"), .what = (void*)&PORTB, },
};

int main(void) {

	DDRB |= _BV(PB0);

	while(1) {
		PORTB ^= _BV(PB0);
		_delay_ms(100);
	}

	return 0;
}
