/*
 * spi.c - Pi J1850 Interface SPI communications 
 */

#include "spi.h"

void spi_init_slave(void) {
    //Initialize ring buffers
    uint8_t i;
    for(i=0; i<2; i++) {
		rx_buf[i].start = 0;
		rx_buf[i].end   = rx_buf[i].start;
		tx_buf[i].start = 0;
		tx_buf[i].end   = tx_buf[i].start;
		
		rx_buf[i].buffer[0] = 0;
		tx_buf[i].buffer[0] = 0;
	}
	
    DDRB |= (1<<PINB4);               //MISO as OUTPUT
    SPCR = (1<<SPE) | (1<<SPIE);       //Enable SPI && interrupt enable bit
    SPDR = 0;
}

ISR(SPI_STC_vect) {
    uint8_t rec_byte = SPDR;
    uint8_t bytes;
    last_spi_byte_tmr = timer_10ms;

	switch(spi_status) {
		case 0x00: //Received a byte while idle
			if(rec_byte <= 4) spi_status = rec_byte;
			//Leave SPDR alone to echo command
			break;
		case 0x01: //Commanded to send switch state
			SPDR = sw_state;
			spi_status = 0xFF;
			break;
		case 0x02: //Read out ACC power state
			SPDR = ((PINB & (1<<PINB0)) != 0) + (((PINC & (1<<PINC5)) != 0)<<1);
			spi_status = 0xFF;
			break;
		case 0x03: //Commanded to read out J1850 buffer 0
			bytes = ring_buf_bytes((ring_buf_t *)&rx_buf[0]);
			SPDR = bytes;
			
			if(bytes) spi_status = 0x10;
			else spi_status = 0x00;
			break;
		case 0x04: //Commanded to read out J1850 buffer 1
			bytes = ring_buf_bytes((ring_buf_t *)&rx_buf[1]);
			SPDR = bytes;
			
			if(bytes) spi_status = 0x11;
			else spi_status = 0x00;
			break;
		case 0x10: //Read out J1850 buffer 0
			SPDR = ring_buf_pop((ring_buf_t *)&rx_buf[0]);
			if(ring_buf_empty((ring_buf_t *)&rx_buf[0])) spi_status = 0xFF;
			break;
		case 0x11: //Read out J1850 buffer 1
			SPDR = ring_buf_pop((ring_buf_t *)&rx_buf[1]);
			if(ring_buf_empty((ring_buf_t *)&rx_buf[1])) spi_status = 0xFF;
			break;
		case 0xFF: //Done after sending a byte
			SPDR = 0x00;
			spi_status = 0;
			break;
	}
}
