/*
 * spi.h - Pi J1850 Interface SPI communications 
 */

#ifndef __SPI_H__
#define __SPI_H__

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "spi.h"
#include "main.h"
#include "ringbuffer.h"

volatile ring_buf_t rx_buf[2];
volatile ring_buf_t tx_buf[2];	   

volatile uint8_t spi_status;

volatile uint8_t last_spi_byte_tmr;

void spi_init_slave(void);

#endif // __SPI_H__
