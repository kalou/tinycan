#ifndef _CAN_H
#define _CAN_H

#ifdef AVR
#include <avr/io.h>
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/* Can std header:
 *  [id:11][1:RTR][1:IDE][1:RSVD][4:DLC]
 *
 * Can ext header:
 *  [id:11][1:SRE][1:IDE][id:18][RTR:1][r1:1][r2:1][DLC:4]
 *
 *          byte 0   byte 1   byte 2   byte 3   byte 4
 *         76543210|76543210|76543210|76543210|76543210
 *  std:   IIIIIIII IIIRi.DD DD
 *  ext:   IIIIIIII IIISiXXX XXXXXXXX XXXXXXXR ..DDDD
 *
 *   With I=ID0 (11 bits)
 *        R=RTR, S=SRE
 *        i=IDE - ext:i=1, std:i=0
 *        .: reserved
 *        X=extended id
 *        D=DLC, data length code
 *
 * Either followed by:
 *  [DATA][CRC:15][ACK:2][EOF:7] // [IFS:7]
 *
 * Id 29 bits: IIIIIIIIIIIXXXXXXXXXXXXXXXXXX
 *             (   ID0   )(      ID1       )
 *                 11bits       18bits
 */

#define can_rtr(x) (can_ide(x) ? x[3] & 1 : !!(x[1] & (1<<4)))
#define can_ide(x) (!!(x[1] & (1<<3)))

#define can_dlc(x) (can_ide(x) ? (x[4] & 0b00111100) >> 2 : \
		                 ((x[1] & 0b00000011) << 2 | \
				  (x[2] & 0b11000000) >> 6))

#define can_len(x) (can_rtr(x) ? 0 : can_dlc(x))

#define can_id0(x) ((uint32_t)(x[0]&0xff) << 3 | (x[1]&0xe0) >> 5)
#define can_id1(x) (((uint32_t)(x[1]&7) << 15 | \
	             (uint32_t)(x[2]&0xff) << 7 | \
		     (x[3]&0xfe) >> 1))
#define can_id(x) (can_ide(x) ? (((uint32_t) can_id0(x) << 18) | can_id1(x)) : \
		   can_id0(x))

// Last bit of arbitration
#define can_arbitration_bit(x) (can_ide(x) ? 32 : 12)
// First bit of data
#define can_data_bit(x) (can_arbitration_bit(x) + 6)
// First bit of crc
#define can_crc_bit(x) (can_data_bit(x) * 8*can_len(x))

#define ntohs(x) ((x & 0xff) << 8 | (x & 0xff00) >> 8)

#define bit_set(x, bit) !!((x)[bit>>3] & (1<<(7-(bit&7))))
#define set_bit(x, bit, v) do { \
	if (v) \
		*((char *) x + (bit >> 3)) |= (1<<(7-(bit&7))); \
	else \
		*((char *) x + (bit >> 3)) &= ~(1<<(7-(bit&7))); \
} while(0)

typedef enum {
	INTERMISSION,
	OVERLOAD,
	SUSPEND,
	BUS_IDLE,
	READING_HEADER,
	READING_DATA,
	READING_CRC,
	READING_ACK,
	READING_EOF,
	WRITING_HEADER,
	WRITING_DATA,
	WRITING_CRC,
	WRITING_CRC_DELIMITER,
	WRITING_ACK,    // This is part of the receiver, but
			// is an active transmission state.
	WRITING_EOF,
	WRITING_ERROR,
	READING_ERROR,
	READING_ERROR_DELIMITER,
	BUS_OFF
} can_state_t;

#define can_transmitting() (can.state >= WRITING_HEADER && \
		can.state <= WRITING_EOF)

#define can_receiving() (can.state >= READING_HEADER || \
		can.state <= READING_EOF)

typedef enum {
	NO_ERROR,
	STUFFING_ERROR,
	BIT_ERROR,
	CRC_ERROR,
	ACKNOWLEDGMENT_ERROR,
	FORM_ERROR,
	// The below are API errors
	LOST_ARBITRATION,
} can_error_t;

typedef enum {
	DOMINANT = 0,
	RECESSIVE = 1
} can_bus_state_t;

typedef struct {
	uint32_t id;
	bool     rtr;
	// len could be set with no data, for RTR frames.
	uint8_t  len;
	char *data;
} can_header_t;

typedef struct {
        uint32_t write_cnt; /* count of write attempts */
        uint32_t ack_rcvd;  /* count of msg ack received */
        uint32_t sof_rcvd;  /* count of times SOF from idle */

        uint32_t ack_error; /* we didn't get an ack */
        uint32_t msg_rcvd;  /* fully received to EOF */
        uint32_t msg_sent;  /* fully sent to EOF */

        uint32_t arbitration_lost;

        uint32_t form_error;  /* someone on bus wrote unexpected bit */
        uint32_t stuff_error; /* we read unexpected stuffing */
        uint32_t crc_error; /* crc we read doesn't match */
        uint32_t bit_error; /* we failed to write bit: state change didn't appear */

        uint32_t unexpected; /* everything else for debug purposes */

        uint32_t bus_off; /* how many times we entered bus_off */
} can_stats_t;

typedef struct _can_controller {
	can_state_t state;

	can_stats_t stats;

	can_error_t error;
	can_state_t error_state; // Where we failed

	// Bus status (read, write)
	can_bus_state_t next_tx;
	can_bus_state_t last_sample;

	// This buffer used for reads and
	// writes. Arbitration can happen
	// in-place.
	char header[5]; // For extended can, 38 bits
	char data[8];   // Up to 8 bytes of data
	uint16_t wire_crc;       // And a 15 bits CRC

	bool     read_available; // Received a full frame
	bool     write_pending;  // User wants us to start 
				 // a new frame.

	uint8_t  bit;            // Which bit we're at
	uint16_t crc;            // Local computation of CRC
				 // when receiving data.



	// Statistics
	uint16_t  history;	 // History of bits.
	uint8_t   state_cnt;
	uint8_t   stuff_cnt;

#define stuffing_active() ((can.state >= READING_HEADER && can.state <= READING_CRC) || \
		(can.state >= WRITING_HEADER && can.state <= WRITING_CRC))

#define n_msk(x) ((1<<x)-1)
#define ones(x, cnt) ((x & n_msk(cnt)) == n_msk(cnt))
#define zeros(x, cnt) ((x & n_msk(cnt)) == 0)
#define stuff_kind() !(can.history&1)
#define needs_stuffing() (can.stuff_cnt >= 5 && (ones(can.history, 5) || \
		          zeros(can.history, 5)))

#define state_recessive_bits(n) (can.state_cnt >= n && ones(can.history, n))
#define state_dominant_bits(n) (can.state_cnt >= n && zeros(can.history, n))

	uint8_t adj_faster;
	uint8_t adj_slower;

	uint8_t txerr_budget;		// Error counters
	uint8_t rxerr_budget;

#define min(a,b) (a<b?a:b)
#define max(a,b) (a>b?a:b)
#define maybe_inc(x, v) do { if (x < 255-v) { x = x+v; } else { x = 255; }} while(0)
#define maybe_dec(x, v) do { if (x > v) { x = x-v; } else { x = 0; }} while(0)
#define dec_rxerr(v) maybe_dec(can.rxerr, v)
#define dec_txerr(v) maybe_dec(can.txerr, v)
#define inc_rxerr(v) maybe_inc(can.rxerr, v)
#define inc_txerr(v) maybe_inc(can.txerr, v)
#define inc_err(err, v) do { if (transmitter_error(err)) { inc_txerr(v); } \
		             if (receiver_error(err)) { inc_rxerr(v); }; } while(0)

#define bus_off() (can.txerr_budget == 0 || can.rxerr_budget == 0)
#define error_passive() (can.txerr_budget < 128 || can.rxerr_budget < 128)

} can_controller_t;

extern can_controller_t can;

#define CAN_RJW 4

/* Port definitions: we need 2 ports to talk to
 * the MCP2561, eventually 3 */
#ifdef AVR
#define DD_STBY DDB1
#define P_STBY  PB1

#define DD_TXD  DDB2
#define P_TXD   PB2

#define DD_RXD  DDB4
#define ADC_RXD ADC2D
#define P_RXD   PB4

#define INT_RXD PCINT4

/* We'll poll RXD, and write to TXD/STBY */
#define TXD_high() do {PORTB |= (1<<P_TXD);} while(0)
#define TXD_low()  do {PORTB &= ~(1<<P_TXD);} while(0)

#define TXD_set(v) do {if (v) TXD_high(); else TXD_low(); } while(0)

#define STBY_high() do {PORTB |= (1<<P_STBY);} while(0)
#define STBY_low() do {PORTB &= ~(1<<P_STBY);} while(0)
#define RXD ((PINB >> PINB4) & 1)
#endif

#if defined(DEBUG)
# ifdef AVR
#  define debug(...)
#  define DD_DEBUG (1 << DDB3)
#  define P_DEBUG (1 << PB3)
#  define DEBUG_high() do { PORTB |= P_DEBUG; } while(0)
#  define DEBUG_low() do { PORTB &= ~P_DEBUG; } while(0)
#  define blink() do { PORTB ^= P_DEBUG; } while(0)
# else /* NO AVR */
#  define debug(...) do { printf(__VA_ARGS__); } while(0)
#  define blink()
#  define DEBUG_high()
#  define DEBUG_low()
# endif
#else /* NO DEBUG */
#  define debug(...)
#  define DD_DEBUG 0
#  define P_DEBUG 0
#  define blink()
#  define DEBUG_high()
#  define DEBUG_low()
#endif


void sync_to_head(void);
can_state_t can_read_bit();
void can_set_tx(can_state_t);
int _do_send(uint32_t, char*, int);
uint16_t wdt_set(uint16_t);
void can_loop();


// API:
void can_init();
// Put the CPU to sleep.
void sleep();
// Put the CPU to sleep, preparing for CAN wakeup.
void sleep_rx_wakeup();
// Sleep-efficient sleeping function.
void can_delay_ms(uint16_t ms);
// Compute CRC
uint16_t compute_crc(void *buf, int len);
// Receive either type.
can_header_t *can_receive(uint16_t timeout_ms);
// Send a data frame. Returns 0 if successfull,
// -1 if we received an arbitration frame,
// in which case caller should call can_receive()
// to read the frame.
int can_send(uint32_t id, char *data, int dlen);

// Send a RTR frame.
int can_request(uint32_t id, int dlen);

#endif
