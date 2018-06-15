/*
 * j1850.c - J1850 interface
 */

#include "j1850.h"

/*
 * Service the pin change ISR and direct it to the proper bus
 */
ISR(PCINT2_vect) {
    uint8_t const tmr_cnt = TCNT2;
    uint8_t const bus_pin[2] = { J1850_BUS0_PIN_REG & J1850_BUS0_PIN_MSK, 
								 J1850_BUS1_PIN_REG & J1850_BUS1_PIN_MSK };
	
	if(J1850_BUS0_PCINT_REG & J1850_BUS0_PCINT_MSK) {
		if(bus_pin[0] ^ j1850_bus[0].last_pin) {
			j1850_bus[0].last_pin = bus_pin[0];
			
			uint8_t const state = j1850_bus[0].state;
			j1850_msg_buf_t *const rx_msg = (j1850_msg_buf_t *)&j1850_bus[0].rx_buf[j1850_bus[0].rx_msg_end];
			uint8_t const delta = tmr_cnt - j1850_bus[0].ltmr;
			j1850_bus[0].ltmr = tmr_cnt;
			
			if(state == 0 && bus_pin[0]) {
				//Transition away from idle
				j1850_bus[0].state ++;
			}
			else if(state == 1 && !bus_pin[0]) {
				//Check for SOF
				if(delta > RX_SOF_MAX || delta < RX_SOF_MIN) {
					j1850_bus[0].state = 0;
				}
				else {
					j1850_bus[0].state ++;
					rx_msg->bit_ptr = 0;
					rx_msg->byte_ptr = 0;
					rx_msg->bytes = 0;
				}
			}
			else if(state == 2) {
				//Receive data bits
				if(rx_msg->byte_ptr < 12 && delta > RX_SHORT_MIN && delta < RX_LONG_MAX) {
					//Setup EOD interrupt
					J1850_BUS0_OCR_REG = tmr_cnt + RX_EOD_MIN;
					J1850_BUS0_OCF_REG |= J1850_BUS0_OCF_MSK;
					J1850_BUS0_OCIE_REG |= J1850_BUS0_OCIE_MSK;
					
					uint8_t *const rx_byte = &rx_msg->buf[rx_msg->byte_ptr];
					
					*rx_byte <<= 1;
					if((bus_pin[0] && delta > RX_LONG_MIN) || (!bus_pin[0] && delta < RX_SHORT_MAX)) {
						//Passive 1
						*rx_byte |= 1;
					}
					rx_msg->bit_ptr ++;
					
					if(rx_msg->bit_ptr == 8) {
						rx_msg->bit_ptr = 0;
						rx_msg->byte_ptr ++;
						rx_msg->bytes ++;
					}
				}
				else {
					//We've started the 13th byte or the pulse was too short/long, something went wrong
					J1850_BUS0_OCIE_REG &= ~J1850_BUS0_OCIE_MSK;
					j1850_bus[0].state = 0;
				}
			}
		}
	}
	
	if(J1850_BUS1_PCINT_REG & J1850_BUS1_PCINT_MSK) {
		if(bus_pin[1] ^ j1850_bus[1].last_pin) {
			j1850_bus[1].last_pin = bus_pin[1];
			
			uint8_t const state = j1850_bus[1].state;
			j1850_msg_buf_t *const rx_msg = (j1850_msg_buf_t *)&j1850_bus[1].rx_buf[j1850_bus[1].rx_msg_end];
			uint8_t const delta = tmr_cnt - j1850_bus[1].ltmr;
			j1850_bus[1].ltmr = tmr_cnt;
			
			if(state == 0 && bus_pin[1]) {
				//Transition away from idle
				j1850_bus[1].state ++;
			}
			else if(state == 1 && !bus_pin[1]) {
				//Check for SOF
				if(delta > RX_SOF_MAX || delta < RX_SOF_MIN) {
					j1850_bus[1].state = 0;
				}
				else {
					j1850_bus[1].state ++;
					rx_msg->bit_ptr = 0;
					rx_msg->byte_ptr = 0;
					rx_msg->bytes = 0;
				}
			}
			else if(state == 2) {
				//Receive data bits
				if(rx_msg->byte_ptr < 12 && delta > RX_SHORT_MIN && delta < RX_LONG_MAX) {
					//Setup EOD interrupt
					J1850_BUS1_OCR_REG = tmr_cnt + RX_EOD_MIN;
					J1850_BUS1_OCF_REG |= J1850_BUS1_OCF_MSK;
					J1850_BUS1_OCIE_REG |= J1850_BUS1_OCIE_MSK;
					
					uint8_t *const rx_byte = &rx_msg->buf[rx_msg->byte_ptr];
					
					*rx_byte <<= 1;
					if((bus_pin[1] && delta > RX_LONG_MIN) || (!bus_pin[1] && delta < RX_SHORT_MAX)) {
						//Passive 1
						*rx_byte |= 1;
					}
					rx_msg->bit_ptr ++;
					
					if(rx_msg->bit_ptr == 8) {
						rx_msg->bit_ptr = 0;
						rx_msg->byte_ptr ++;
						rx_msg->bytes ++;
					}
				}
				else {
					//We've started the 13th byte or the pulse was too short/long, something went wrong
					J1850_BUS1_OCIE_REG &= ~J1850_BUS1_OCIE_MSK;
					j1850_bus[1].state = 0;
				}
			}
		}
	}
}

/*
 * J1850 channel timer compare interrupts
 */
 
ISR(TIMER2_COMPA_vect) {
	uint8_t tmr_cnt = TCNT2;
	
	if(j1850_bus[0].state == 2) {
		//Received EOD
		J1850_BUS0_OCIE_REG &= ~J1850_BUS0_OCIE_MSK;
		j1850_bus[0].new_msg = 1;
		j1850_bus[0].rx_msg_end++;
		if(j1850_bus[0].rx_msg_end == J1850_MSG_BUF_SIZE) j1850_bus[0].rx_msg_end = 0;
		j1850_bus[0].state = 0;
	}
	else if(j1850_bus[0].state == 10) {
		//Try to send something
		j1850_bus[0].state = 11;
		j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].bit_ptr = 0;
		j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].byte_ptr = 0xFF;
		J1850_BUS0_PORT_REG &= ~J1850_BUS0_PORT_MSK;
		J1850_BUS0_OCR_REG = tmr_cnt + TX_IFS;
		J1850_BUS0_OCF_REG |= J1850_BUS0_OCF_MSK;
		J1850_BUS0_OCIE_REG |= J1850_BUS0_OCIE_MSK;
	}
	else if(j1850_bus[0].state == 11) {
		//Waiting for IFS
		uint8_t delay;
		uint8_t pin;
		
		pin = J1850_BUS0_PIN_REG & J1850_BUS0_PIN_MSK;
		
		if(pin) {
			//Bus isn't passive
			delay = TX_IFS;
		}
		else {
			//Successfully waited for IFS
			j1850_bus[0].state = 12;
			J1850_BUS0_PORT_REG |= J1850_BUS0_PORT_MSK;
			delay = TX_SOF;
		}
		
		J1850_BUS0_OCR_REG = tmr_cnt + delay;
		J1850_BUS0_OCF_REG |= J1850_BUS0_OCF_MSK;
		J1850_BUS0_OCIE_REG |= J1850_BUS0_OCIE_MSK;
	}
	else if(j1850_bus[0].state == 12) {
		//Sending bits
		uint8_t pin;
		pin = J1850_BUS0_PIN_REG & J1850_BUS0_PIN_MSK;
		
		if(pin) J1850_BUS0_PORT_REG &= ~J1850_BUS0_PORT_MSK;
		else J1850_BUS0_PORT_REG |= J1850_BUS0_PORT_MSK;
		
		if(j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].bit_ptr) j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].bit_ptr --;
		else if(j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].bytes) {
			j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].bytes --;
			j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].byte_ptr ++;
			j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].bit_ptr = 7;
		}
		else {
			//Done sending bits
			j1850_bus[0].state = 0;
			
			j1850_bus[0].tx_msg_start++;
			if(j1850_bus[0].tx_msg_start == J1850_MSG_BUF_SIZE) j1850_bus[0].tx_msg_start = 0;
			
			J1850_BUS0_OCIE_REG &= ~J1850_BUS0_OCIE_MSK;
			return;
		}
		
		uint8_t delay;
		
		if(!(j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].buf[j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].byte_ptr] & 0x80) != !pin) delay = TX_SHORT;
		else delay = TX_LONG;
		
		j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].buf[j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_start].byte_ptr] <<= 1;
		
		J1850_BUS0_OCR_REG = tmr_cnt + delay;
		J1850_BUS0_OCF_REG |= J1850_BUS0_OCF_MSK;
		J1850_BUS0_OCIE_REG |= J1850_BUS0_OCIE_MSK;
	}
}

ISR(TIMER2_COMPB_vect) {
	uint8_t tmr_cnt = TCNT2;
	
	if(j1850_bus[1].state == 2) {
		//Received EOD
		J1850_BUS1_OCIE_REG &= ~J1850_BUS1_OCIE_MSK;
		j1850_bus[1].new_msg = 1;
		j1850_bus[1].rx_msg_end++;
		if(j1850_bus[1].rx_msg_end == J1850_MSG_BUF_SIZE) j1850_bus[1].rx_msg_end = 0;
		j1850_bus[1].state = 0;
	}
	else if(j1850_bus[1].state == 10) {
		//Try to send something
		j1850_bus[1].state = 11;
		j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].bit_ptr = 0;
		j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].byte_ptr = 0xFF;
		J1850_BUS1_PORT_REG &= ~J1850_BUS1_PORT_MSK;
		J1850_BUS1_OCR_REG = tmr_cnt + TX_IFS;
		J1850_BUS1_OCF_REG |= J1850_BUS1_OCF_MSK;
		J1850_BUS1_OCIE_REG |= J1850_BUS1_OCIE_MSK;
	}
	else if(j1850_bus[1].state == 11) {
		//Waiting for IFS
		uint8_t delay;
		uint8_t pin;
		
		pin = J1850_BUS1_PIN_REG & J1850_BUS1_PIN_MSK;
		
		if(pin) {
			//Bus isn't passive
			delay = TX_IFS;
		}
		else {
			//Successfully waited for IFS
			j1850_bus[1].state = 12;
			J1850_BUS1_PORT_REG |= J1850_BUS1_PORT_MSK;
			delay = TX_SOF;
		}
		
		J1850_BUS1_OCR_REG = tmr_cnt + delay;
		J1850_BUS1_OCF_REG |= J1850_BUS1_OCF_MSK;
		J1850_BUS1_OCIE_REG |= J1850_BUS1_OCIE_MSK;
	}
	else if(j1850_bus[1].state == 12) {
		//Sending bits
		uint8_t pin;
		pin = J1850_BUS1_PIN_REG & J1850_BUS1_PIN_MSK;
		
		if(pin) J1850_BUS1_PORT_REG &= ~J1850_BUS1_PORT_MSK;
		else J1850_BUS1_PORT_REG |= J1850_BUS1_PORT_MSK;
		
		if(j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].bit_ptr) j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].bit_ptr --;
		else if(j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].bytes) {
			j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].bytes --;
			j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].byte_ptr ++;
			j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].bit_ptr = 7;
		}
		else {
			//Done sending bits
			j1850_bus[1].state = 0;
			
			j1850_bus[1].tx_msg_start++;
			if(j1850_bus[1].tx_msg_start == J1850_MSG_BUF_SIZE) j1850_bus[1].tx_msg_start = 0;
			
			J1850_BUS1_OCIE_REG &= ~J1850_BUS1_OCIE_MSK;
			return;
		}
		
		uint8_t delay;
		
		if(!(j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].buf[j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].byte_ptr] & 0x80) != !pin) delay = TX_SHORT;
		else delay = TX_LONG;
		
		j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].buf[j1850_bus[1].tx_buf[j1850_bus[1].tx_msg_start].byte_ptr] <<= 1;
		
		J1850_BUS1_OCR_REG = tmr_cnt + delay;
		J1850_BUS1_OCF_REG |= J1850_BUS1_OCF_MSK;
		J1850_BUS1_OCIE_REG |= J1850_BUS1_OCIE_MSK;
	}
}

/*
 * Starts the interrupt system sending out the next message in the buffer
 */
void j1850_send_packet(uint8_t bus) {
	j1850_bus[bus].state = 10;
	
	if(bus) {
		J1850_BUS1_OCR_REG = TCNT2 + 1;
		J1850_BUS1_OCF_REG |= J1850_BUS1_OCF_MSK;
		J1850_BUS1_OCIE_REG |= J1850_BUS1_OCIE_MSK;
	}
	else {
		J1850_BUS0_OCR_REG = TCNT2 + 1;
		J1850_BUS0_OCF_REG |= J1850_BUS0_OCF_MSK;
		J1850_BUS0_OCIE_REG |= J1850_BUS0_OCIE_MSK;
	}
}

/*
 * Calculates an appropriate CRC for a given message
 */
uint8_t j1850_crc(uint8_t *msg_buf, int8_t nbytes) {
	uint8_t crc_reg=0xff,poly,byte_count,bit_count;
	uint8_t *byte_point;
	uint8_t bit_point;

	for (byte_count=0, byte_point=msg_buf; byte_count<nbytes; ++byte_count, ++byte_point)
	{
		for (bit_count=0, bit_point=0x80 ; bit_count<8; ++bit_count, bit_point>>=1)
		{
			if (bit_point & *byte_point)	// case for new bit = 1
			{
				if (crc_reg & 0x80)
					poly=1;	// define the polynomial
				else
					poly=0x1c;
				crc_reg= ( (crc_reg << 1) | 1) ^ poly;
			}
			else		// case for new bit = 0
			{
				poly=0;
				if (crc_reg & 0x80)
					poly=0x1d;
				crc_reg= (crc_reg << 1) ^ poly;
			}
		}
	}
	return ~crc_reg;	// Return CRC
}

/*
 * Initialize all the J1850 stuff
 */
void j1850_init(void) {
	J1850_BUS0_DDRPORT_REG |= J1850_BUS0_PORT_MSK;
	J1850_BUS0_DDRPIN_REG &= ~J1850_BUS0_PIN_MSK;
	J1850_BUS0_PCINT_REG |= J1850_BUS0_PCINT_MSK;
	
	J1850_BUS1_DDRPORT_REG |= J1850_BUS1_PORT_MSK;
	J1850_BUS1_DDRPIN_REG &= ~J1850_BUS1_PIN_MSK;
	J1850_BUS1_PCINT_REG |= J1850_BUS1_PCINT_MSK;
    
    TCCR2A = 0;
    //1/64 prescaler
    TCCR2B = (1<<CS21) | (1<<CS20);
    
    //Enable PCI
    PCICR = (1<<PCIE2);
}
