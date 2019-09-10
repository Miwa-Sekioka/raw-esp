/* TODO:
 *  - dump rx data from interrupt handler into ring buffer first.
 *
 *  - using preallocated "flat" tx buffer with size around 8-16KB
 *    would be more efficient than malloc-ing packets all the time,
 *    and it'd also make mem fragmentation impossible.
 *    It'd also make impossible to reorder packets, but we don't do
 *    it anyway. It'd also require modified COBS encoder that writes
 *    directly into buffer.
 */

#include "osapi.h"
#include "eagle_soc.h"
#include "c_types.h"
#include "driver/uart.h"
#include "ets_sys.h"
#include "user_interface.h"
#include "mem.h"
//#include "missing_declarations.h"

#include "comm.h"
#include "misc.h"
#include "crc16.h"
#include "comm_spi.h"

#define COMM_TASK_PRIO USER_TASK_PRIO_0

#define UART0   0
#define UART1   1

#define DO_RX 80
#define DO_TX 81

// Shift beginnig of the buffer so payload is aligned.
// This way message headers can be cast to structure directly.
#define BUF_ALIGN_OFFSET (__BIGGEST_ALIGNMENT__ - 1)

/* ------------------------------------------------------------------ send */

#define TX_RING_BUFFER_SIZE 32
#define TX_RING_BUFFER_MASK (TX_RING_BUFFER_SIZE - 1)

struct transmitter {
	uint8_t *bufs[TX_RING_BUFFER_SIZE];
	size_t   buf_lens[TX_RING_BUFFER_SIZE];
	volatile uint16_t buf_read_i;
	volatile uint16_t buf_write_i;

	volatile bool task_pending;
	uint32_t dropped_packets;
	size_t idx_in_buf;
};

struct transmitter transmitter_uart0;
static comm_callback_t m_cb = NULL;

static void ICACHE_FLASH_ATTR
transmitter_init(struct transmitter *t)
{
	t->buf_read_i = 0;
	t->buf_write_i = 0;
	t->idx_in_buf = 0;
	t->task_pending = false;
	t->dropped_packets = 0;
}


static void transmitter_wake_task(struct transmitter *t)
{
	// We don't want to put a lot of messages into task queue since it's
	// quite small. So we use a flag to tell if we've already put task into
	// queue and it's not dispatched yet.
	if (t->task_pending)
		return;

	ets_intr_lock();
	if (!t->task_pending) {
		t->task_pending = true;
		system_os_post(COMM_TASK_PRIO, DO_TX, 0);
	}
	ets_intr_unlock();
}


// This function can be called from different contexts. It disables
// interrupts in the critical section.
static void ICACHE_FLASH_ATTR
transmitter_push(struct transmitter *t, uint8_t *data, size_t len, size_t prio)
{
	// precheck if we'll queue packet
	size_t buf_used = t->buf_write_i - t->buf_read_i;
	if (buf_used == TX_RING_BUFFER_SIZE)
		goto drop;

	size_t buf_free = TX_RING_BUFFER_SIZE - buf_used;
	size_t heap_free = system_get_free_heap_size(); // PERF: how much does it cost?
	
	switch (prio) {
	case COMM_TX_PRIO_LOW:
		if ((heap_free < 20000) || (buf_free < 15)) goto drop; break;
	case COMM_TX_PRIO_MEDIUM:
		if ((heap_free < 10000) || (buf_free < 10)) goto drop; break;
	case COMM_TX_PRIO_HIGH:
		if ((heap_free <  2000) || (buf_free <  1)) goto drop; break;
	default:;
	}

	// prepare data
	uint8_t *encoded = os_malloc(len);
	if (!encoded)
		return;

  //copy data
  os_memcpy(encoded, data, len);

	// put into ring buffer
	ets_intr_lock();
	if (t->buf_write_i - t->buf_read_i == TX_RING_BUFFER_SIZE) {
		ets_intr_unlock();
		os_free(encoded);
		goto drop;
	}

	size_t i = t->buf_write_i & TX_RING_BUFFER_MASK;
	t->bufs[i] = encoded;
	t->buf_lens[i] = len;
	t->buf_write_i++;
	ets_intr_unlock();

	transmitter_wake_task(t);
	return;

 drop:
	t->dropped_packets += 1;
}


// This function must not be called from several places at once.
// We call it only from comm task.
static void transmitter_send(struct transmitter *t)
{
	ets_intr_lock();
	t->task_pending = false;
	ets_intr_unlock();

	size_t fifo_free_n = spi_send_ready() ? 1: 0;
	if (fifo_free_n == 0) {
		//uart0_tx_intr_enable();
		return;
	}

	// TODO: I'm not sure if it's edge triggered or value triggered,
	// enable it for now to be safe.
	while (fifo_free_n && (t->buf_read_i != t->buf_write_i)) {
		size_t buf_idx = t->buf_read_i & TX_RING_BUFFER_MASK;
		uint8_t *buf = t->bufs[buf_idx];
		size_t len = t->buf_lens[buf_idx];
		size_t idx = t->idx_in_buf;

    spi_send_data(buf, len);
    idx = len;
    fifo_free_n = 0;
 
		if (idx != len) {
			t->idx_in_buf = idx;
			break;
		} else {
			t->buf_read_i++;
			t->idx_in_buf = 0;
			os_free(buf);
		}
	}

	if ((!fifo_free_n) && (t->buf_read_i != t->buf_write_i)) {
		//uart0_tx_intr_enable();
	}
}


/* ------------------------------------------------------------------ receive */
/*
struct decoder {
	struct cobs_decoder cobs;
	uint8_t buf[BUF_ALIGN_OFFSET + COBS_ENCODED_MAX_SIZE(MAX_MESSAGE_SIZE)];

	uint32_t proto_errors;
	uint32_t crc_errors;
	comm_callback_t cb;
};

struct decoder dec_uart0;
*/

static inline void ICACHE_FLASH_ATTR
on_frame_received(uint8_t *data, size_t len)
{
	uint16_t crc_msg;
	uint16_t crc_calc;

	if (len < 3) {
    os_printf("len err\n");	
		return;
	}

	crc_calc = crc16_block(data, len - 2);
	os_memcpy(&crc_msg, data + len - 2, 2);
	if (crc_calc != crc_msg) {
    os_printf("crc err\n");	
		return;
	}

	if (m_cb){
	  //os_printf("<<:%02x:%d", data[0], len-3);
    m_cb(data[0], data + 1, len - 3);
	}
}

/*
static inline void ICACHE_FLASH_ATTR
decoder_init(struct decoder *dec, comm_callback_t cb)
{
	cobs_decoder_init(
		&dec->cobs,
		dec->buf + BUF_ALIGN_OFFSET, sizeof(dec->buf) - BUF_ALIGN_OFFSET,
		decoder_check_and_dispatch_cb, dec);
	dec->proto_errors = 0;
	dec->crc_errors = 0;
	dec->cb = cb;
}*/

/*
static inline void ICACHE_FLASH_ATTR
decoder_put_data(struct decoder *dec, void *data, size_t len)
{
	cobs_decoder_put(&dec->cobs, data, len);
}*/

static void do_rx()
{
	uint8_t *data;
	uint32_t length;
	spi_get_recv_data(&data, &length);
	if(data != NULL){
		on_frame_received(data, length);
		spi_recv_buffer_clear();
  }
}


/* --------------------------------------------------------------- irq, task */

static void comm_task(os_event_t *e)
{
	switch (e->sig) {
  	case DO_RX: do_rx(); break;
	  case DO_TX: transmitter_send(&transmitter_uart0); break;
	  default: COMM_ERR("unknown task variant");
	}
}

/* ---------------------------------------------------------------- interface */

uint8_t comm_loglevel = 0;

void ICACHE_FLASH_ATTR
comm_set_loglevel(uint8_t level)
{
	comm_loglevel = level;
}


// need 1 for DO_RX and 2 for DO_TX
os_event_t comm_queue[3];

void ICACHE_FLASH_ATTR
comm_init(comm_callback_t cb) {
  m_cb = cb;
	transmitter_init(&transmitter_uart0);

	system_os_task(comm_task, COMM_TASK_PRIO, comm_queue, ARRAY_SIZE(comm_queue));
	spi_init();
}


void ICACHE_FLASH_ATTR
comm_get_stats(uint32_t *rx_errors,
               uint32_t *rx_crc_errors,
               uint32_t *dropped_packets) {
	//*rx_errors = dec_uart0.proto_errors + dec_uart0.crc_errors;
	//*rx_crc_errors = dec_uart0.crc_errors;
	*dropped_packets = transmitter_uart0.dropped_packets;
}


void ICACHE_FLASH_ATTR
comm_send(uint8_t type, void *data, size_t n, size_t prio)
{
	uint8_t buf[n + 3];

	buf[0] = type;
	os_memcpy(buf + 1, data, n);

	uint16_t crc = crc16_block(buf, n + 1);
	buf[n+1] = crc & 0xff;
	buf[n+2] = (crc >> 8) & 0xff;

	transmitter_push(&transmitter_uart0, buf, n + 3, prio);
}


void ICACHE_FLASH_ATTR
comm_send_ctl(uint8_t type, void *data, size_t n)
{
	comm_send(type, data, n, COMM_TX_PRIO_HIGH);
}


void ICACHE_FLASH_ATTR
comm_send_status(uint8_t s)
{
	comm_send_ctl(MSG_STATUS, &s, sizeof(s));
}

void ICACHE_FLASH_ATTR
spi_send_end_handler()
{
	transmitter_wake_task(&transmitter_uart0);
}

void ICACHE_FLASH_ATTR
spi_recv_handler(uint8_t *data, uint32_t length)
{
  system_os_post(COMM_TASK_PRIO, DO_RX, 0);
}

