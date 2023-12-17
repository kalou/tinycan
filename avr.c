#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "can.h"

#define DD_LED (1 << DDB0)
#define P_LED (1 << PB0)
#define LED_high() do { PORTB |= P_LED; } while(0)
#define LED_low() do { PORTB &= ~P_LED; } while(0)

void show_errors()
{
	if (can.error) {
		for (int i =0; i < can.error; i++) {
			LED_high();
			can_delay_ms(250);
			LED_low();
			can_delay_ms(400);
		}

		can_delay_ms(1000);

		for (int i =0; i < can.error_state; i++) {
			LED_high();
			can_delay_ms(250);
			LED_low();
			can_delay_ms(400);
		}

		can_delay_ms(1000);
	}
}

int main(void)
{
	int err;
	can_header_t *c;
	DDRB |= DD_LED;
	can_init();

	for (;;) {
		char debug[4];
		can_header_t *c = can_receive(0);

		if (c) {
			if (c->id == 0x125)
				adjust_osccal(1);
			if (c->id == 0x126)
				adjust_osccal(-1);

			debug[0] = 1;
			debug[1] = 0x23;
			debug[2] = 1;
			debug[3] = OSCCAL;
			LED_high();
			can_send(0x123, debug, 4);
			LED_low();
		}
	}
}
