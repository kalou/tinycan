#ifdef AVR
#include <avr/interrupt.h>
#include <avr/sleep.h>
#endif

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h> /* memset */
#include <stdint.h>

#include "can.h"

can_controller_t can = { 0 };

uint16_t update_crc(uint16_t crc, int bit)
{
	crc ^= (bit << 14);
	crc <<= 1;
	if (crc & 0x8000) {
		crc ^= 0x4599;
	}
	uint16_t ret = crc & 0x7fff;
	return ret;
}

uint16_t compute_crc(void *buf, int len)
{
	uint16_t crc = 0;
	for (int bit = 0; bit < 8*len; bit++) {
		crc = update_crc(crc, bit_set((char *) buf, bit));
	}
	return crc;
}

void can_set_state(can_state_t new_state) {
	/* Reset CRC if we're done working
	 * with it.
	 */
	if ((can.state == READING_CRC) ||
	    (can.state == WRITING_CRC)) {
		can.wire_crc = 0;
		can.crc = 0;
	}

	/* Reset bit count in state, except if
	 * we lost arbitration. */
	if (!(can.state == WRITING_HEADER &&
	    new_state == READING_HEADER)) {
		can.bit = 0;
		can.state_cnt = 0;
	}

	can.state = new_state;
}

void update_counters(int bit)
{
	if (stuffing_active()) {
		can.stuff_cnt++;
	} else {
		// Start of frame counts as one.
		can.stuff_cnt = 1;
	}
	can.state_cnt++;
	can.history <<= 1;
	can.history |= bit;
}

volatile uint16_t wdt_time_ms = 0;

#ifdef AVR
void adjust_osccal(int8_t v)
{
	if ((OSCCAL + v) & 0x7f)
		OSCCAL += v;
}


ISR(PCINT0_vect)
{
}

// Put the CPU to sleep. Caller needs to setup the
// wakeup condition.
void sleep()
{
	// Sleep, power down
	MCUCR = (1<<SE) | (1<<SM1);
	// Sleep, ADC on
	//MCUCR = (1<<SE) | (1<<SM0);

	sei();
	sleep_cpu();
}

// Put CPU to sleep, xceiver in standby mode, and
// wakeup on its signal that data is ready.
void sleep_rx_wakeup()
{
	// Put transceiver in standby mode.
	STBY_high();
	// Set interrupt so we wake up on the RXD pin.
	PCMSK |= (1<<INT_RXD);

	sleep();

	// Clear interrupt.
	PCMSK &= ~(1<<INT_RXD);
	// Put transceiver back on.
	STBY_low();
}

ISR(WDT_vect)
{
	// See Table 8-3.
	uint8_t wd_config = ((WDTCR & (1<<WDP3)) >> 2) | (WDTCR & 7);
	wdt_time_ms += (16 << wd_config);
}

/* Set watchdog for the largest chunk we can handle within requested
 * duration, <16 to disable, and return the value.
 */
uint16_t wdt_set(uint16_t ms)
{
	WDTCR = 0;
	for (int wd_config = 9; wd_config >= 0; wd_config--) {
		if (ms >= 16<<wd_config) {
			// Set watchdog config (see Table 8-3.)
			WDTCR = (1<<WDIE) | 
				(!!(wd_config&1) << WDP0) |
				(!!(wd_config&2) << WDP1) |
				(!!(wd_config&4) << WDP2) |
				(!!(wd_config&8) << WDP3);
			return (16<<wd_config);
		}
	}

	return 0;
}

/* wdt_reset() is WDR instruction */

void can_delay_ms(uint16_t ms)
{
	while(ms > 16) {
		wdt_time_ms = 0;
		wdt_set(ms);
		while (!wdt_time_ms)
			// Watchdog wakes up CPU, leverage sleeping.
			sleep();
		ms -= wdt_time_ms;
	}

	do {
		// 20 bits at 20kbps -> 1ms
		for (uint8_t i = 0; i < 20; i++) {
			while(TCNT0);
			while(!TCNT0);
		}
	} while(ms--);
}

// We use timer compares for sample point and clock duty cycle.
#define wrapped() (TIFR & (1<<OCF0A))
#define clear_wrapped() do { TIFR |= (1<<OCF0A); } while(0)

#define sample_point() (TIFR & (1<<OCF0B))
#define clear_sample_point() do { TIFR |= (1<<OCF0B); } while(0)

// This signals pin change detection.
#define edge() (GIFR & (1<<PCIF))
#define clear_edge() do { (GIFR |= (1<<PCIF)); } while(0)

// Synchronize with either edge or clock cycle, which
// will represent head of bit time.
inline void sync_to_head()
{
	while(!wrapped() && RXD == can.last_sample);
	if (!wrapped()) {
		// If we detect edge while waiting for end of
		// frame, our clock is running late.
		// TODO: RJW, OSCCAL, ...
		int err = OCR0A - TCNT0;
		TCNT0 = 0;
		can.adj_faster = err;
	} else {
		clear_wrapped();
	}
}

// Wait until sample point and return the bit value.
can_state_t can_read_bit()
{
	can.last_sample = RXD;
	while (!sample_point() && RXD == can.last_sample);
	if (!sample_point()) {
		// If we detect an edge before sample point,
		// after SYNC segment, our clock might be running
		// too fast.
		// TODO: RJW, OSCCAL, ...
		if (TCNT0 > 2) {
			int err = TCNT0 - 2;
			TCNT0 = 0;
			can.adj_slower = err;
		}
	} else {
		clear_sample_point();
	}

	can.last_sample = RXD;

	return can.last_sample;
}

void can_set_tx(can_state_t v)
{
	can.last_sample = v;
	TXD_set(v);
}
#endif /* AVR */

void can_init()
{
#ifdef AVR
	// Set output on TX, STBY, DEBUG
	DEBUG_low();
	TXD_high(); // Recessive
	STBY_low(); // No standby

	PORTB |= 1<<P_TXD; // Recessive TXD=1
	DDRB |= (1<<DD_STBY) | (1<<DD_TXD) | (DD_DEBUG);

	// Make sure input buffers are not disabled for RX
	DIDR0 &= ~(1<<ADC_RXD);

	CLKPR = (1<<CLKPCE); // Enable changing clock speed
	CLKPR = 0;           // Divider = 1 => Fcpu at 8Mhz

	// Enable port change interrupts for wakeup
	GIMSK |= (1<<PCIE);

	// Config for 20kbps CAN bus - faster will require optimizing
	// processing time. For example current CRC takes 20 cycles,
	// which is 5us or 20% of a 50kbps bit at 8mhz.
	//
	// See: CAN20.pdf, AN1798.pdf
	//
	// With prescaler /8, CAN timer clock is 1Mhz, one clock tick
	// is 1us.
	//
	TCCR0B = 1 << CS01; // Divide by 8 (AVR: Table 11-6)

	// Set CTC mode, clearing counter at OCRA (see datasheet Table 11-5)
	// In order to compare, OCR0A needs to be set *after* we set CTC mode.
	TCCR0A = (1<< WGM01);
	
	// Sync seg could be ~2us, giving 25 TQ/frame.
	// Prop seg is ~2x bus delay = 2*cable+tx+rx, estimated at approx 1us
	// For 20kbps: duty 50us, sample point at 20us
	OCR0A = 50;
	OCR0B = 20;
	// Equivalence with PHASE_SEG1/PHASE_SEG2 adjustments:
	//  - Reducing OCR0A is reducing PHASE_SEG2
	//  - Increasing OCR0B and OCR0A is increasing PHASE_SEG1
#endif /* AVR */
	memset(&can, '\x0', sizeof(can));
}

void handle_error(can_error_t err)
{
	debug("ERROR %d\n", err);

	switch(err) {
		case STUFFING_ERROR:
		case CRC_ERROR:
		case FORM_ERROR:
			inc_rxerr(1);
			break;
		case BIT_ERROR:
		case ACKNOWLEDGMENT_ERROR:
			inc_txerr(1);
		default:
			break;
	}

	can.error = err;
	can.error_state = can.state;

	if (bus_off()) {
		can_set_state(BUS_OFF);
	} else if (error_passive()) {
		can_set_state(READING_ERROR);
	} else {
		can.next_tx = DOMINANT;
		can_set_state(WRITING_ERROR);
	}
}

void can_loop()
{
	sync_to_head();

	if(needs_stuffing()) {
		can_state_t stuff_kind = stuff_kind();
		if (can_transmitting()) {
			can_set_tx(stuff_kind);
		}

		uint8_t bit = can_read_bit();
		update_counters(bit);

		if (bit != stuff_kind) {
			debug("stuff needed, but can_read != %d\n", stuff_kind);
			handle_error(STUFFING_ERROR);
			return;
		}

		sync_to_head();
	}

	// Both safeguard and init time convenience
	if (can.state == INTERMISSION)
		can.next_tx = RECESSIVE;

	can_set_tx(can.next_tx);

	/* Go to sample point, reading the resulting value */
	uint8_t bit = can_read_bit();
	update_counters(bit);

	debug("state=%d bit=%d writing=%d << %d\n", can.state, can.bit, can.next_tx, bit);

	if (can_transmitting() && (bit != can.next_tx)) {
		if (can.state == WRITING_HEADER &&
		    can.bit <= can_arbitration_bit(can.header)) {
			// We are writing recessive and reading
			// a dominant bit, switch to reading the
			// incoming frame.
			debug("Lost arbitration\n");
			can.write_pending = false;
			can_set_state(READING_HEADER);
		} else if (can.state == READING_ACK) {
			// Our CRC is confirmed.
		} else {
			// This is a bit error. Handle at next
			// bit.
			handle_error(BIT_ERROR);
			return;
		}
	}

	switch(can.state) {
		case INTERMISSION:
			if (state_recessive_bits(3)) {
				if (error_passive()) {
					can_set_state(SUSPEND);
				} else {
					can_set_state(BUS_IDLE);
				}
			} else if (bit == DOMINANT) {
				// 9.1 Protocol Modifications
				// Interpret this as SOF:
				if (can.state_cnt == 3) {
					if (can.write_pending)
						can_set_state(WRITING_HEADER);
					else
						can_set_state(READING_HEADER);
				} else {
					can.next_tx = DOMINANT;
					can_set_state(OVERLOAD);
				}
			}
			break;
		case OVERLOAD:
			/* Normally should only allow for 2 overload frames,
			 * but transceiver will take care of that
			 */
			if (state_dominant_bits(6)) {
				can.next_tx = RECESSIVE;
			}
			if (state_recessive_bits(8)) {
				can_set_state(INTERMISSION);
			}
			break;
		case SUSPEND:
			/* Wait 8 more bits before we consider ourselves
			 * bus_idle.
			 */
			if (state_recessive_bits(8)) {
				can_set_state(BUS_IDLE);
			}
			if (bit != DOMINANT)
				break;
			/* Fall through if a new message starts: */
		case BUS_IDLE:
			if (bit == DOMINANT) {
				debug("Received start of frame\n");
				if (can.write_pending) {
					can_set_state(WRITING_HEADER);
				} else {
					can_set_state(READING_HEADER);
				}
			} else if (can.write_pending) {
				debug("Starting frame\n");
				can.next_tx = DOMINANT;
				can_set_state(WRITING_HEADER);
			}
			break;
		case WRITING_HEADER:
			can.next_tx = bit_set(can.header, can.bit);
			can.crc = update_crc(can.crc, can.next_tx);
			debug("Header %d >> %d\n", can.bit, can.next_tx);
			if (++can.bit >= can_data_bit(can.header)) {
				can.bit = 0;
				if (can_len(can.header))
					can_set_state(WRITING_DATA);
				else
					can_set_state(WRITING_CRC);
			}
			break;
		case WRITING_DATA:
			can.next_tx = bit_set(can.data, can.bit);
			can.crc = update_crc(can.crc, can.next_tx);
			debug("Data %d >> %d\n", can.bit, can.next_tx);
			if (++can.bit >= 8 * can_len(can.header)) {
				can.bit = 0;
				debug("computed CRC %d\n", can.crc);
				can_set_state(WRITING_CRC);
			}
			break;
		case WRITING_CRC:
			can.next_tx = !!(can.crc & (1<<(14-can.bit)));
			debug("CRC %d >> %d\n", can.bit, can.next_tx);
			if(++can.bit >= 16) {
				can.next_tx = RECESSIVE;
				can_set_state(WRITING_CRC_DELIMITER);
			}
			break;
		case WRITING_CRC_DELIMITER:
			can_set_state(READING_ACK);
			break;
		case READING_ACK:
			if (bit == DOMINANT) {
				dec_txerr(1);
				can_set_state(WRITING_EOF);
			} else {
				handle_error(ACKNOWLEDGMENT_ERROR);
			}
			break;
		case READING_HEADER:
			debug("Header %d << %d\n", can.bit, bit);
			can.crc = update_crc(can.crc, bit);
			if (can.bit == 0) {
				memset(can.header, 0, sizeof(can.header));
				memset(can.data, 0, sizeof(can.data));
			}
			set_bit(&can.header, can.bit, bit);
			if (++can.bit >= can_data_bit(can.header)) {
				can.bit = 0;
				if (can_len(can.header)) {
					can_set_state(READING_DATA);
				} else  {
					can_set_state(READING_CRC);
				}
			}
			break;
		case READING_DATA:
			debug("Data %d << %d\n", can.bit, bit);
			can.crc = update_crc(can.crc, bit);
			set_bit(can.data, can.bit, bit);
			if (++can.bit >= 8 * can_len(can.header)) {
				can.bit = 0;
				can_set_state(READING_CRC);
			}
			break;
		case READING_CRC:
			debug("CRC %d << %d\n", can.bit, bit);
			set_bit(&can.wire_crc, can.bit, bit);
			if (++can.bit >= 16) {
				can.wire_crc = ntohs(can.wire_crc) >> 1;
				if (can.wire_crc == can.crc) {
					dec_rxerr(1);
					can.next_tx = DOMINANT;
				} else {
					debug("Wire CRC %d != expected %d\n",
							can.wire_crc, can.crc);
				}
				can_set_state(WRITING_ACK);
			}
			break;
		case WRITING_ACK:
			// We just read the CRC delimiter, we're currently
			// sending the "ack slot". Set next state to recessive.
			can.next_tx = RECESSIVE;
			can_set_state(READING_EOF);
			break;
		case READING_EOF:
			// We're reading the ACK delimiter or one of the
			// 7 recessive bit of EOF:
			if (bit != RECESSIVE) {
				handle_error(FORM_ERROR);
			}
			// That's when an eventual CRC error is reported:
			if (can.wire_crc != can.crc) {
				handle_error(CRC_ERROR);
			}
			if (state_recessive_bits(8)) {
				can_set_state(INTERMISSION);
				can.read_available = true;
			}
			break;
		case WRITING_EOF:
			// Write the ack delimiter, followed by EOF
			// (7 recessive bits).
			if (state_recessive_bits(8)) {
				can_set_state(INTERMISSION);
				can.write_pending = false;
			}
			break;
		case BUS_OFF:
			// If we observe 11 recessive bits 128 times,
			// clear the slate. Since we came here by having
			// one of the counters reach 255, this can be
			// piggybacked on them.
			if (state_recessive_bits(11)) {
				dec_rxerr(1);
				dec_txerr(1);
			}
			if (can.txerr < 128 && can.rxerr < 128) {
				can.txerr = can.rxerr = 0;
				can_set_state(INTERMISSION);
			}
			break;
		case WRITING_ERROR:
			if (state_dominant_bits(6)) {
				can.next_tx = RECESSIVE;
				can_set_state(READING_ERROR_DELIMITER);
			}
			break;
		case READING_ERROR:
			if (state_recessive_bits(6))
				can_set_state(READING_ERROR_DELIMITER);
			break;
		case READING_ERROR_DELIMITER:
			if (state_recessive_bits(8)) {
				can_set_state(INTERMISSION);
			}
	}
}

void can_fill_header(char *buf, uint32_t id, int rtr, int dlen)
{
	if (id > 0xFFF) {
		buf[0] = (id >> 21) & 0xff;
		buf[1] = ((id >> 18) & 7) << 5 | (0 << 4) /* srr */ | \
		       (1 << 3) /* ide */ | ((id >> 15) & 7);
		buf[2] = (id >> 7) & 0xff;
		buf[3] = (id & 0x7f) << 1 | rtr;
		buf[4] = (dlen & 15) << 2;
	} else {
		buf[0] = (id >> 3) & 0xff;
		buf[1] = (id & 7) << 5 | (rtr << 4) | /*ide*/ (0 << 3);
		buf[1] |= (dlen >> 2) & 3;
		buf[2] = (dlen & 3) << 6;
	}
}

int _do_send(uint32_t id, char *data, int dlen)
{
	if (data) {
		can_fill_header(can.header, id, 0, dlen);
		memcpy(can.data, data, dlen);
	} else
		can_fill_header(can.header, id, 1, dlen);

	debug("Can send id=%x ide=%d, data len %d\n", can_id(can.header),
			can_ide(can.header), dlen);

	// Bus blocks retrying.
	can.write_pending = true;
	do {
		can_loop();
	} while(can.write_pending || can.state != BUS_IDLE);

	// Arbitration: we read a frame instead.
	// Caller should read the frame and retry.
	if (can.read_available)
		return -LOST_ARBITRATION;
	return 0;
}

int can_send(uint32_t id, char *data, int dlen)
{
	return _do_send(id, data, dlen);
}

int can_request(uint32_t id, int dlen)
{
	return _do_send(id, NULL, dlen);
}

/* Receive with timeout. */
can_header_t *can_receive(uint16_t timeout_ms)
{
	static can_header_t res;
	uint8_t idle_count = 0;

	wdt_time_ms = 0;
	wdt_set(timeout_ms);
	while(!can.read_available) {
		can_loop();
		if (can.state == BUS_IDLE) {
			if (wdt_time_ms) {
				timeout_ms -= wdt_time_ms;
				wdt_time_ms = 0;
				wdt_set(timeout_ms);
			}
			// 1ms
			if (idle_count++ > 20) {
				if (timeout_ms < 16) {
					if (!timeout_ms--)
						return NULL;
				} else {
					sleep_rx_wakeup();
				}
				idle_count = 0;
			}
		}
	}

	can.read_available = false;

	res.id = can_id(can.header);
	res.rtr = can_rtr(can.header);
	res.len = can_dlc(can.header);
	res.data = can.data;
	return &res;
}
