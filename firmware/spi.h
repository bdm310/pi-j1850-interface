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
#include "j1850.h"

volatile uint8_t spi_status;
volatile uint8_t spi_active;

volatile uint8_t last_spi_byte_tmr;
volatile uint8_t spi_bytes;

volatile j1850_msg_buf_t bus0_rx[J1850_MSG_BUF_SIZE];
volatile uint8_t bus0_rx_start;
volatile uint8_t bus0_rx_end;
volatile j1850_msg_buf_t bus1_rx[J1850_MSG_BUF_SIZE];
volatile uint8_t bus1_rx_start;
volatile uint8_t bus1_rx_end;

void spi_init_slave(void);

#endif // __SPI_H__
