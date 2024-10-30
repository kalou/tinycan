#include <stdlib.h>
#include <string.h>

#include "can.h"

void print_can_state()
{
	printf("Can state is %d, state_cnt %d - writes: %d, acks_rcvd: %d\n"
	       "\tsof_rcvd:%d msg_rcvd: %d\n"
               "\tack_errors:%d msg_sent: %d\n"
	       "\tarbitration_lost:%d\n"
	       "\tform_errors:%d stuff_error:%d crc_error:%d\n"
	       "\tbit_error:%d unexp:%d\n"
	       "\ttxerr_budget:%d rxerr_budget:%d\n",
	       can.state, can.state_cnt, can.stats.write_cnt, can.stats.ack_rcvd,
	       can.stats.sof_rcvd, can.stats.msg_rcvd,
	       can.stats.ack_error, can.stats.msg_sent,
	       can.stats.arbitration_lost, can.stats.form_error,
	       can.stats.stuff_error, can.stats.crc_error,
	       can.stats.bit_error, can.stats.unexpected,
	       can.txerr_budget, can.rxerr_budget);

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
static bool simulate_ack = true;

static can_state_t mock_tx = 1;
static int mock_tx_stream[200] = { 1 };

can_state_t can_read_bit()
{
	// If we're writing low, we should read it back.
	if (mock_tx == 0)
		return 0;
	// Eventually simulate receiving ack from remote station.
	if (simulate_ack && can.state == READING_ACK)
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

#define OK(x) do {printf("OK " x "\n");} while(0)
#define FAIL(x) do { printf("FAIL " x "\n"); exit(1); } while(0)
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
		FAIL("incorrect ID");

	if (f->len != exp_len)
		FAIL("incorrect DLC");

	if (exp_data && memcmp(f->data, exp_data, f->len))
		FAIL("bad data");

	if (exp_data == NULL && f->rtr != 1)
		FAIL("expected RTR frame");

	OK();
}

void test_can_send(uint32_t id, int len, char *data, int n_bits, int *bits)
{
	int failcnt = 0;
	mock_idx = 0;
	mock_max_idx = 0;
	mock_rx_stream = NULL;
	simulate_ack = true;

	_do_send(id, data, len);
	printf("wrote(%d bits)=", mock_idx);
	for (int i = 0; i < mock_idx; i++) {
		printf("%d", mock_tx_stream[i]);
		if (bits && i < n_bits && mock_tx_stream[i] != bits[i]) {
			printf("!");
			failcnt++;
		}
		printf(",");
	}
	printf("\n");
	print_can_state();

	if (failcnt)
		FAIL("Unexpected bits");
	OK();
}

/* Sends "loses" to 0x123, but receives "wins" to 0x121 */
void test_arbitration()
{
	can_init();
	debug("Test arbitration loss\n");
	mock_idx = 0;
	mock_max_idx = 90;
	simulate_ack = false;
	mock_rx_stream = (int[]) {1,1,1,1,1,
		0,//SOF
		0,0,1,0,0,1,0,0,0,0,1, // Id 0x121
		0,0,0,0,1,0,0,0,
		1,1,1,0,1,1,1,0,
		1,1,0,1,0,0,1,0,
		1,1,0,1,1,1,0,0,
		1,1,1,0,0,1,1,0,
		1,1,1,0,1,0,0,1,
		1,1,1,0,0,1,1,1,
		1,1,1,1,1,1,1,1,
		1,1,1,1,1,1,};

	int ret = _do_send(0x123, "loses", 5);
	printf("wrote(%d bits), ret=%d\n", mock_idx, ret);
	print_can_state();

	if (ret != -LOST_ARBITRATION)
		FAIL("Expected arbitration loss");

	if (can_dlc(can.header) != 4 || memcmp(can.data, "wins", 4))
		FAIL("Expected data=`wins` length=4");
	OK();
}

void test_bus_off_recovers()
{
	can_init();
	debug("Test bus_off recovers\n");
	mock_idx = 0;
	mock_max_idx = 1;
	simulate_ack = false;
	mock_rx_stream = (int[]) {1,1,1,1}; // max_idx will return more 1s

	can.state = BUS_OFF;
	can.rxerr_budget = 0;
	can.txerr_budget = 0;
	do {
		can_loop();
		print_can_state();
	} while (can.state != INTERMISSION);
}

#define TEST_receive(name, exp_id, exp_len, exp_data, ...) do { \
	int name##_bits[] = { __VA_ARGS__ }; \
	can_init(); \
	printf("Test receive " #name "\n"); \
	test_can_receive(exp_id, exp_len, exp_data, sizeof(name##_bits)/sizeof(int), name##_bits); \
} while(0)

#define TEST_send(name, id, len, data, ...) do { \
	int name##_bits[] = { __VA_ARGS__ }; \
	can_init(); \
	printf("Test send " #name "\n"); \
	test_can_send(id, len, data, sizeof(name##_bits)/sizeof(int), name##_bits); \
} while(0)

int main(void)
{

	can_init();

	TEST_send(WritingWins, 0x121, 4, "wins");

	/* TODO: known to fail because we compute CRC ahead
	 * of checking bit write status.
	 *
	 * test_arbitration();
	 *
	 */
	test_bus_off_recovers();


	TEST_send(WritingLong, 0x123, 7, "\0\0\0\0\0\0\0",
			1,1,1,1,1,//1,
			0, // SOF
			0,0,1,0,0,1,0,0,0,1,1, // ID
			0,0,0, // RTR, IDE, RSV
			0,1,1,1, // LEN
			// 56 bits (7*8bytes) of 0 with stuffing:
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
			0,0,0,0,0,1,
			// CRC
			0,0,1,1,0,0,1,1,1,1,1,0/*stuff*/, // stuff bit
			1,1,1,1,0, // CRC? One bit off??
			1, // Delim
			1, //ACK
			1,1,1,1,1,1);

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
	TEST_send(RTR, 0x123, 0, NULL, 1,1,1,1,1,
			0, // SOF
			0,0,1,0,0,1,0,0,0,1,1, // ID
			1,0,0, // RTR=1
			0,0,0, // LEN[...]
			1, // stuff
			0, // LEN
			0,0,1,1,0,1,1,1,0,0,1,1,1,0,1, // CRC
			1, // Delimiter
			1, // Ack slot
			1, // Delimiter
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

	TEST_send(HighId_29bits, 0x1f01000d, 0, NULL);
	TEST_receive(HighId_29bits, 0x1f01000d, 0, NULL, 1,1,1,1,1,
			1,1,1,1,1,0,1,1,1,1,1,0,0,0,0,0,1,0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1,1,0,1,1,0,0,0,0,0,1,0,1,1,1,0,1,0,0,0,0,0,1,0,0,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1);

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


	// RTR to 0x2a
	TEST_send(RTR2a, 0x2a, 0, NULL, 1,1,1,1,1,//1,
			0, /* SOF */
			0,0,0,0,1/*stuff*/,0,1,0,1,0,1,0, /* Addr */
			1, /* RTR */
			0, // IDE
			0, // RSV
			0,0,0,/* stuff */1,0, // Len=0
			1,0,1,1,1,1,1,/*stuff*/0,1,0,1,1,0,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1);

	TEST_receive(RTR2a, 0x2a, 0, NULL, 1,1,1,1,1,1,
			0, /* SOF */
			0,0,0,0,1 /*stuff*/,0,1,0,1,0,1,0, /* Addr */
			1, /* RTR */
			0, // IDE
			0, // RSV
			0,0,0,/* stuff */1,0, // Len=0
			1,0,1,1,1,1,1,/*stuff*/0,1,0,1,1,0,1,1,1,
			1,1,1,1,1,1,1,1,1,1,1,1);

	TEST_receive(StuffingSOF, 0x2a, 4, "\x12\x12\x12\x12", 1,1,1,1,1,1,
			0, // SOF
			0,0,0,0, // Id[...]
			1, // Stuff
			0,1,0,1,0,1,0,// Id[...]
			0,0,0, // RTR=0 IDE=0 RSV=0
			0, // LEN = 0100
			1, // STUFF
			1,0,0, // LEN = 0100
			// 0b000110010: \x12 with stuff
			0,0,0, // DATA[0]
			1, // STUFF
			1,0,0,1,0, // DATA[0]
			0,0,0,1,0,0,1,0, // DATA[1]
			0,0,0,1,0,0,1,0, // DATA[2]
			0,0,0,1,0,0,1,0, // DATA[3]
			0,0,1,1,1,0,0,1,0,1,0,0,0,1,0, // CRC
			1, // CRC delimiter
			1, // ACK BIT
			1, // ACK delimiter
			1,1,1,1,1,1,1,1,1);

	// Stuffing following start of frame.
	TEST_send(StuffingSOF, 0x2a, 4, "\x12\x12\x12\x12",
			1,1,1,1,1,//1,
			0, // SOF
			0,0,0,0, // Id[...]
			1, // Stuff
			0,1,0,1,0,1,0,// Id[...]
			0,0,0, // RTR=0 IDE=0 RSV=0
			0, // LEN = 0100
			1, // STUFF
			1,0,0, // LEN = 0100
			// 0b000110010: \x12 with stuff
			0,0,0, // DATA[0]
			1, // STUFF
			1,0,0,1,0, // DATA[0]
			0,0,0,1,0,0,1,0, // DATA[1]
			0,0,0,1,0,0,1,0, // DATA[2]
			0,0,0,1,0,0,1,0, // DATA[3]
			0,0,1,1,1,0,0,1,0,1,0,0,0,1,0, // CRC
			1, // CRC delimiter
			1, // ACK BIT
			1, // ACK delimiter
			1,1,1,1,1,1,1,1,1);

	// While stuck on real bus?
	TEST_send(WhyStuck, 0xfff, 6, "\x12\x12\x12\x12\x12\x12");
}
