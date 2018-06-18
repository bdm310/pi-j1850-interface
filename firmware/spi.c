/*
 * spi.c - Pi J1850 Interface SPI communications 
 */

#include "spi.h"

void spi_init_slave(void) {
    DDRB |= (1<<PINB4);               //MISO as OUTPUT
    SPCR = (1<<SPE) | (1<<SPIE);       //Enable SPI && interrupt enable bit
    SPDR = 0;
    
    bus0_rx_start = 0;
    bus0_rx_end = bus0_rx_start;
    bus1_rx_start = 0;
    bus1_rx_end = bus1_rx_start;
}

ISR(SPI_STC_vect) {
	uint8_t byte;
    uint8_t rec_byte = SPDR;
    last_spi_byte_tmr = timer_10ms;
    spi_active = 1;

	switch(spi_status) {
		case 0x00: //Received a byte while idle
			if(rec_byte <= 7) spi_status = rec_byte;
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
			if(bus0_rx_start != bus0_rx_end) {
				spi_bytes = bus0_rx[bus0_rx_start].bytes;
			}
			else {
				spi_bytes = 0;
			}
			SPDR = spi_bytes;
			
			if(spi_bytes) spi_status = 0x10;
			else spi_status = 0x00;
			break;
		case 0x04: //Commanded to read out J1850 buffer 1
			if(bus1_rx_start != bus1_rx_end) {
				spi_bytes = bus1_rx[bus1_rx_start].bytes;
			}
			else {
				spi_bytes = 0;
			}
			SPDR = spi_bytes;
			
			if(spi_bytes) spi_status = 0x11;
			else spi_status = 0x00;
			break;
		case 0x05: //Prepare to receive J1850 message 0
			spi_bytes = SPDR;
			spi_status = 0x12;
			break;
		case 0x06: //Prepare to receive J1850 message 1
			spi_bytes = SPDR;
			spi_status = 0x13;
			break;
		case 0x07: //Prepare to receive listen headers
			spi_bytes = SPDR;
			j1850_listen_bytes = 0;
			if(SPDR) spi_status = 0x14;
			else spi_status = 0x00;
			break;
		case 0x10: //Read out J1850 buffer 0
			byte = bus0_rx[bus0_rx_start].bytes - spi_bytes;
			SPDR = bus0_rx[bus0_rx_start].buf[byte];
			spi_bytes --;
			if(spi_bytes == 0) {
				bus0_rx_start++;
				if(bus0_rx_start == J1850_MSG_BUF_SIZE) bus0_rx_start = 0;
				spi_status = 0xFF;
			}
			break;
		case 0x11: //Read out J1850 buffer 1
			byte = bus1_rx[bus1_rx_start].bytes - spi_bytes;
			SPDR = bus1_rx[bus1_rx_start].buf[byte];
			spi_bytes --;
			if(spi_bytes == 0) {
				bus1_rx_start++;
				if(bus1_rx_start == J1850_MSG_BUF_SIZE) bus1_rx_start = 0;
				spi_status = 0xFF;
			}
			break;
		case 0x12: //Receive J1850 message 0
			j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].bytes] = SPDR;
			j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].bytes++;
			spi_bytes--;
			if(spi_bytes == 0x0) {
				uint8_t prev_end = j1850_bus[0].tx_msg_end;
				j1850_bus[0].tx_msg_end++;
				if(j1850_bus[0].tx_msg_end == J1850_MSG_BUF_SIZE) j1850_bus[0].tx_msg_end = 0;
				
				//Buffer overflow
				if(j1850_bus[0].tx_msg_end == j1850_bus[0].tx_msg_start) j1850_bus[0].tx_msg_end = prev_end;
				
				spi_status = 0xFF;
			}
			break;
		case 0x13: //Receive J1850 message 1
			j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_end].buf[j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_end].bytes] = SPDR;
			j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_end].bytes++;
			spi_bytes--;
			if(spi_bytes == 0x0) {
				uint8_t prev_end = j1850_bus[1].tx_msg_end;
				j1850_bus[1].tx_msg_end++;
				if(j1850_bus[1].tx_msg_end == J1850_MSG_BUF_SIZE) j1850_bus[1].tx_msg_end = 0;
				
				//Buffer overflow
				if(j1850_bus[1].tx_msg_end == j1850_bus[1].tx_msg_start) j1850_bus[1].tx_msg_end = prev_end;
				
				spi_status = 0xFF;
			}
			break;
		case 0x14: //Receive listen headers
			j1850_listen_headers[j1850_listen_bytes] = SPDR;
			j1850_listen_bytes++;
			spi_bytes--;
			if(spi_bytes == 0x0) spi_status = 0xFF;
			break;
		case 0xFF: //Done after sending a byte
			SPDR = 0x00;
			spi_status = 0;
			break;
	}
}

