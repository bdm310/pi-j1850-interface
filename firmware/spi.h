/*
 * spi.h - Pi J1850 Interface SPI communications 
 */

#ifndef __SPI_H__
#define __SPI_H__

#include <avr/io.h>

uint8_t spi_active;

void spi_init_slave(void);
void spi_process(uint8_t tmr_10ms);
inline int8_t spi_tx_push(uint8_t byte);

#define SPI_BUF_SIZE 65

#define MISO_DDR DDRB 
#define MISO_MSK (1<<PINB4)

#endif // __SPI_H__
