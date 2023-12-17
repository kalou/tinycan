#include <stdlib.h>
#include <string.h>

#include "can.h"

void print_can_state()
{
	printf("Can state is %d, rxerr:%d txerr:%d\n", can.state,
			can.rxerr, can.txerr);
	printf("Bits read %d, data: ", can.bit);
	for (int i = 0; i < can_len(can.header); i++) {
		printf("[%x] ", (unsigned char) can.data[i]);
	}
	printf("\nCrc: wire=%d, computed=%d\n", can.wire_crc, can.crc);
	printf("Msg id 0x%x rtr:%d ide:%d len:%d\n", can_id(can.header),
			can_rtr(can.header), can_ide(can.header),
			can_len(can.header));

	printf("Frame dump:\n");
	for (unsigned char *p = (unsigned char *) &can.header;
			p < (unsigned char *) &can.header + can_data_bit(can.header)/8 + 
			   can_len(can.header) + 1; p++) {
		printf("[%x] ", *p);
	}
	printf("\n");
}

static int *mock_rx_stream = NULL;
static int mock_idx = 0, mock_max_idx = 0;

static can_state_t mock_tx = 1;
static int mock_tx_stream[200] = { 1 };

can_state_t can_read_bit()
{
	int m;

	if (mock_tx == 0)
		return 0;
	if (can.state == READING_ACK)
		return 0;
	if ((mock_idx >= mock_max_idx) ||
	    (mock_rx_stream == NULL))
		return 1;

	return mock_rx_stream[mock_idx];
}

void can_set_tx(can_state_t v)
{
	mock_tx = v;
	if (mock_idx < 200)
		mock_tx_stream[mock_idx] = v;
}

void sync_to_head()
{
	mock_idx++;
}

void sleep_rx_wakeup()
{
}

extern uint16_t wdt_time_ms;
uint16_t wdt_set(uint16_t ms)
{
	wdt_time_ms = ms;
	return ms;
}

#define FAIL(x) do { printf("FAIL " x); exit(1); } while(0)
void test_can_receive(uint32_t exp_id, int exp_len, char *exp_data, int n_bits, int *bits)
{
	mock_idx = 0;
	mock_max_idx = n_bits;
	mock_rx_stream = bits;

	can_header_t *f = can_receive(1);
	print_can_state();

	if (f == NULL)
		FAIL("receive failed");

	if (f->id != exp_id)
		FAIL("incorrect ID\n");

	if (f->len != exp_len)
		FAIL("incorrect DLC\n");

	if (exp_data && memcmp(f->data, exp_data, f->len))
		FAIL("bad data\n");

	if (exp_data == NULL && f->rtr != 1)
		FAIL("expected RTR frame\n");

	printf("OK\n");
}

void test_can_send(uint32_t id, int len, char *data, int n_bits, int *bits)
{
	mock_idx = 0;
	mock_max_idx = n_bits;
	mock_rx_stream = bits;

	_do_send(id, data, len);
	printf("wrote=");
	for (int i = 0; i < mock_idx; i++) {
		printf("%d,", mock_tx_stream[i]);
		if (bits && i < n_bits && mock_tx_stream[i] != bits[i]) {
			printf("!");
		}
	}
	print_can_state();
	printf("\n");
}

#define TEST_receive(name, exp_id, exp_len, exp_data, ...) do { \
	int name##_bits[] = { __VA_ARGS__ }; \
	printf("Test receive " #name "\n"); \
	test_can_receive(exp_id, exp_len, exp_data, sizeof(name##_bits)/sizeof(int), name##_bits); \
} while(0)

#define TEST_send(name, id, len, data, ...) do { \
	int name##_bits[] = { __VA_ARGS__ }; \
	printf("Test send " #name "\n"); \
	test_can_send(id, len, data, sizeof(name##_bits)/sizeof(int), name##_bits); \
} while(0)

int main(void)
{
	TEST_send(WritingLong, 0x123, 7, "\0\0\0\0\0\0\0");
	TEST_receive(ReadingLong, 0x123, 7, "\0\0\0\0\0\0\0", 1,1,1,1,1,
			0, // SOF
			0,0,1,0,0,1,0,0,0,1,1, /* ID */
			0,0,0, /* RTR, IDE, RSV */
			0,1,1,1, /* LEN */
			/* 56 bits (7*8bytes) of 0 with stuffing: */
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,
			0,0,0,0,0,1,0,
			/* CRC */
			0,1,1,0,0,1,1,1,1,1,
			0, /* stuff bit */
			1,1,1,1,0, /* CRC */
			1, /* Delim */
			1/*=ACK*/,1,1,1,1,1,1);

	/* RTR frame */
	TEST_send(RTR, 0x123, "", 0);
	TEST_receive(RTR_11bits, 0x123, 0, NULL, 1,1,1,1,1,1,1,1,1,
			0, // SOF
			0,0,1,0,0,1,0,0,0,1,1, // ID
			1,0,0, // RTR=1
			0,0,0, // LEN[...]
			1, /* stuff */
			0, // LEN
			0,0,1,1,0,1,1,1,0,0,1,1,1,0,1, /* CRC */
			1, /* Delimiter */
			1, /* Ack slot */
			1, /* Delimiter */
			1,1,1,1,1,1,1,1,1);

	/* 11 bits id, no data, good CRC */
	TEST_receive(NoData_11bits, 0x555, 0, "", 1,1,1,1,1,1,1,1,1,1,1,1,1,
			0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,1,0,0,1,1,0,
			0,1,1,1,0,1,0,0,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,-1);

	// can_send(0x123, "\x2a", 1);
	TEST_receive(Data_11bits, 0x123, 1, "\x2a", 1,1,1,1,1,1,1,1,1,1,1,1,1,
			0, /* SOF is first bit: 1 */ 
			0,0,1,0,0,1,0,0,0,1,1, /* Id +11 */
			0,0,0,0,0, /* RTR=0 IDE=0 RSV=0 LEN[..] +5 */
			1, /* STUFFING + 1*/
			0,1, /* LEN[..] +2 */
			0,0,1,0,1,0,1,0, /* DATA +8 */
			0,0,0,0, /*CRC +4 */
			1, /*STUFF +1 */
			0,0,1,1,1,1,0,0,1,0,1, /*CRC +11 */
			1, /* CRC delimiter */
			1, /* ACK BIT is at position 45 */
			1, /* ACK delimiter */
			1,1,1,1,1,1,1, /* EOF */);


	// Stuffing following start of frame.
	TEST_receive(StuffingSOF, 0x2a, 4, "\x12\x12\x12\x12", 1,1,1,1,1,1,
			0,/*SOF*/
			0,0,0,0,/*Id[...]*/
			1,/*Stuff*/
			0,1,0,1,0,1,0,/*Id[...]*/
			0,0,0,0,1,1,0,0,0,0,0,1,1,0,0,1,0,0,0,0,
			1,0,0,1,0,0,0,0,1,0,0,1,0,0,0,0,1,0,0,1,0,0,0,0,1,1,1,0,0,1,0,1,0,
			0,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1);

	//can_send(0x123456cc, "hello", 5);
	//Has a double stuffing edge case at end of data->CRC:
	TEST_receive(LargeData_29bits, 0x123456cc, 5, "hello", 1,1,1,1,1,1,1,1,
			0, /*SOF*/
			1,0,0,1,0,0,0,1,1, /* 0x123 */
			0, /* SRE */
			1, /* IDE */
			0,1,0,0,0,1,0,1,0,1,1,0,1,1,0,0,1,1,0,0, /* 0x456cc */
			0, /* RTR */
			0,0, /* RSVD */
			1, /* Stuff */
			0,1,0,1, /* DLC */
			0,1,1,0,1,0,0,0, /* h */
			0,1,1,0,0,1,0,1, /* e */
			0,1,1,0,1,1,0,0, /* l */
			0,1,1,0,1,1,0,0, /* l */
			0,1,1,0,1,1,1,1, /* o */
			1, /* CRC bit 14 */
			0, /* Stuff */
			0,0,0,0, /* CRC bits 13-10 */
			1, /* Stuff */
			0,0,1,0,0,1,0,0,0,0, /* CRC 9-0 */
			1, /* CRC Delimiter */
			1, /* Ack slot */
			1, /* Ack delimiter */
			1,1,1,1,1,1,1 /* EOF */);
}
