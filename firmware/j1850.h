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

#define J1850_MSG_BUF_SIZE 32

//J1850_OUT
#define J1850_BUS0_PORT_REG PORTD
#define J1850_BUS0_DDRPORT_REG DDRD
#define J1850_BUS0_PORT_MSK (1<<PORTD6)
#define J1850_BUS0_PIN_REG PIND
#define J1850_BUS0_DDRPIN_REG DDRD
#define J1850_BUS0_PIN_MSK (1<<PORTD3)
#define J1850_BUS0_PCINT_REG PCMSK2
#define J1850_BUS0_PCINT_MSK (1<<PCINT19)
#define J1850_BUS0_OCR_REG OCR2A
#define J1850_BUS0_OCF_REG TIFR2
#define J1850_BUS0_OCF_MSK (1<<OCF2A)
#define J1850_BUS0_OCIE_REG TIMSK2
#define J1850_BUS0_OCIE_MSK (1<<OCIE2A)

//J1850_IN
#define J1850_BUS1_PORT_REG PORTB
#define J1850_BUS1_DDRPORT_REG DDRB
#define J1850_BUS1_PORT_MSK (1<<PORTB1)
#define J1850_BUS1_PIN_REG PIND
#define J1850_BUS1_DDRPIN_REG DDRD
#define J1850_BUS1_PIN_MSK (1<<PORTD2)
#define J1850_BUS1_PCINT_REG PCMSK2
#define J1850_BUS1_PCINT_MSK (1<<PCINT18)
#define J1850_BUS1_OCR_REG OCR2B
#define J1850_BUS1_OCF_REG TIFR2
#define J1850_BUS1_OCF_MSK (1<<OCF2B)
#define J1850_BUS1_OCIE_REG TIMSK2
#define J1850_BUS1_OCIE_MSK (1<<OCIE2B)

typedef struct j1850_bus_t j1850_bus_t;
typedef struct j1850_msg_buf_t j1850_msg_buf_t;
typedef struct j1850_event_t j1850_event_t;

struct j1850_msg_buf_t {
	uint8_t buf[12];
	uint8_t bytes;
	uint8_t byte_ptr;
	uint8_t bit_ptr;
};

struct j1850_bus_t {
	uint8_t last_pin;
	uint8_t state;
	uint8_t ltmr;
	uint8_t new_msg;
	j1850_msg_buf_t rx_buf[J1850_MSG_BUF_SIZE];
	uint8_t rx_msg_start;
	uint8_t rx_msg_end;
	j1850_msg_buf_t tx_buf[J1850_MSG_BUF_SIZE];
	uint8_t tx_msg_start;
	uint8_t tx_msg_end;
};

volatile j1850_bus_t j1850_bus[2];

void j1850_init(void);
void j1850_send_packet(uint8_t bus);
uint8_t j1850_crc(uint8_t *msg_buf, int8_t nbytes);

// convert microseconds to counter values
#define ISR_LATENCY 10
#define us2cnt(us) ((unsigned int)(((unsigned long)(us)) / ((1000000L*32L) / (float)((unsigned long)F_CPU / 1L))))

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
