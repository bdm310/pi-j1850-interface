/*
 * j1850.h - J1850 interface
 */
#ifndef __J1850_H__
#define __J1850_H__

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "main.h"
#include "j1850.h"
#include "spi.h"

#define F_CPU 8000000L
/*
typedef struct j1850_bus_t j1850_bus_t;
typedef struct j1850_msg_buf_t j1850_msg_buf_t;

struct j1850_msg_buf_t {
	uint8_t buf[8][12];
	uint8_t bytes[8];
	uint8_t messages;
	uint8_t byte_ptr;
	uint8_t bit_ptr;
};

struct j1850_bus_t {
	uint8_t *port_reg;
	uint8_t *pin_reg;
	uint8_t *ddr_reg;
	uint8_t pin_msk;
	uint8_t last_pin;
	uint8_t state;
	uint8_t ltmr;
	uint8_t new_msg;
	j1850_msg_buf_t rx_buf;
	j1850_msg_buf_t tx_buf;
};

volatile j1850_bus_t j1850_bus[2];
*/

volatile uint8_t j1850_state[2];
// 0 - Idle
// 1 - SOF?
// 2 - Received SOF, receiving bits
//10 - Output?

volatile uint8_t j1850_ltmr[2];
volatile uint8_t j1850_new_msg[2];
volatile uint8_t j1850_rx_buf[2][12];
volatile uint8_t j1850_rx_bytes[2];
volatile uint8_t j1850_rx_byte[2];
volatile uint8_t j1850_rx_bit[2];

volatile uint8_t j1850_tx_buf[2][12];
volatile uint8_t j1850_tx_bytes[2];
volatile uint8_t j1850_tx_byte[2];
volatile uint8_t j1850_tx_bit[2];


volatile uint8_t j1850_last_pin;

void j1850_init(void);
void j1850_send_packet(uint8_t bus);
uint8_t j1850_crc(uint8_t *msg_buf, int8_t nbytes);

// convert microseconds to counter values
#define us2cnt(us) ((unsigned int)((unsigned long)(us) / ((1000000L*32L) / (float)((unsigned long)F_CPU / 1L))))

// define J1850 VPW timing requirements in accordance with SAE J1850 standard
// all pulse width times in us
// transmitting pulse width
#define TX_SHORT	us2cnt(64)		// Short pulse nominal time
#define TX_LONG		us2cnt(128)		// Long pulse nominal time
#define TX_SOF		us2cnt(200)		// Start Of Frame nominal time
#define TX_EOD		us2cnt(200)		// End Of Data nominal time
#define TX_EOF		us2cnt(280)		// End Of Frame nominal time
#define TX_BRK		us2cnt(300)		// Break nominal time
#define TX_IFS		us2cnt(300)		// Inter Frame Separation nominal time

// see SAE J1850 chapter 6.6.2.5 for preferred use of In Frame Respond/Normalization pulse
#define TX_IFR_SHORT_CRC	us2cnt(64)	// short In Frame Respond, IFR contain CRC
#define TX_IFR_LONG_NOCRC us2cnt(128)	// long In Frame Respond, IFR contain no CRC

// receiving pulse width
#define RX_SHORT_MIN	us2cnt(34)	// minimum short pulse time
#define RX_SHORT_MAX	us2cnt(96)	// maximum short pulse time
#define RX_LONG_MIN		us2cnt(96)	// minimum long pulse time
#define RX_LONG_MAX		us2cnt(163)	// maximum long pulse time
#define RX_SOF_MIN		us2cnt(163)	// minimum start of frame time
#define RX_SOF_MAX		us2cnt(239)	// maximum start of frame time
#define RX_EOD_MIN		us2cnt(163)	// minimum end of data time
#define RX_EOD_MAX		us2cnt(239)	// maximum end of data time
#define RX_EOF_MIN		us2cnt(239)	// minimum end of frame time, ends at minimum IFS
#define RX_BRK_MIN		us2cnt(239)	// minimum break time
#define RX_IFS_MIN		us2cnt(280)	// minimum inter frame separation time, ends at next SOF

// see chapter 6.6.2.5 for preferred use of In Frame Respond/Normalization pulse
#define RX_IFR_SHORT_MIN	us2cnt(34)		// minimum short in frame respond pulse time
#define RX_IFR_SHORT_MAX	us2cnt(96)		// maximum short in frame respond pulse time
#define RX_IFR_LONG_MIN		us2cnt(96)		// minimum long in frame respond pulse time
#define RX_IFR_LONG_MAX		us2cnt(163)		// maximum long in frame respond pulse time

#endif // __J1850_H__
