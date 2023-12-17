# tinycan
A minimal 20kbps CANbus controller for ATtiny85 (and others).

## Notes
 * Mostly educational project, slow bit rate. For similar pricing MCUs with CAN support (e.g. PIC18F26Q83) offer better support at 1Mb/s.
 * This however allows to build a small ATtiny85+MCP2561 circuit, using the 8Mhz internal oscillator.

## Features
 * Supports 20kbps, CAN 2.0A 11bits and 2.0B 29 bits identifiers.
 * Error detection (STUFF, CRC) and error timing, ACKs valid frames.
 * Arbitration logic (collision/priority).
 * Extremely simplistic synchronization using OSCCAL (phase segments are not adjusted as specified).
 * Minimalist fault handling.
 * Low current on idle (xceiver standby, sleep on receive).
 
## Usage

```
#include "can.h"

int main(void)
{
        can_init();

        for (;;) {
		// Puts the MCU asleep and the transceiver
		// in standby mode until a message wakes us
		// up, or 10 seconds passed:
                can_header_t *c = can_receive(10000);

		// If we receive a remote request:
                if (c && c->id == 0x123 && c->rtr) {
                        can_send(c->id, "hi", 2);
		}

		// One can also send remote requests:
		can_request(0x2327, 0);
	}
}
```

## TODO, maybe
 * More testing (esp. on bus with nodes) is needed.
 * Need to improve bit timing, sync & resync, and fix some retransmit loops that subside with noise.
 * OVERLOAD is respected, but not used to slow down traffic.
 * Proper lost arbitration hasn't been tested but might work; we bail out to reading header if bit error happens during arbitration phase.
 * Size improvements. Current size is ~4KB.
