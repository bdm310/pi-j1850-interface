/*
 * j1850.c - J1850 interface
 */

#include "j1850.h"

/*
 * Set the proper pin based on bus to the desired state
 */
inline void j1850_set_pin(uint8_t bus, uint8_t state) {
	if(state) *j1850_bus[bus].port_reg |= j1850_bus[bus].port_msk;
	else *j1850_bus[bus].port_reg &= ~j1850_bus[bus].port_msk;
}

/*
 * Set the proper OCR depending on bus
 */
inline void j1850_set_delay(uint8_t *tmr_cnt, uint8_t bus, uint8_t delay) {
	if(bus) {
		OCR2B = *tmr_cnt + delay;
		TIFR2 |= (1<<OCF2B);
		TIMSK2 |= (1<<OCIE2B);
	}
	else {
		OCR2A = *tmr_cnt + delay;
		TIFR2 |= (1<<OCF2A);
		TIMSK2 |= (1<<OCIE2A);
	}
}

/*
 * Turns off the appropriate OCR based on bus
 */
inline void j1850_stop_tmr(uint8_t bus) {
	if(bus) TIMSK2 &= ~(1<<OCIE2B);
	else TIMSK2 &= ~(1<<OCIE2A);
}

inline void j1850_service_pin(uint8_t *tmr_cnt, uint8_t bus) {
	uint8_t pin = *j1850_bus[bus].pin_reg & j1850_bus[bus].pin_msk;
	uint8_t delta = *tmr_cnt - j1850_bus[bus].ltmr;
	j1850_bus[bus].ltmr = *tmr_cnt;
	
	if(j1850_bus[bus].state == 0 && pin) {
		//Transistion away from idle
		j1850_bus[bus].state ++;
	}
	else if(j1850_bus[bus].state == 1 && !pin) {
		//Check for SOF
		if(delta > RX_SOF_MAX || delta < RX_SOF_MIN) {
			j1850_bus[bus].state = 0;
		}
		else {
			j1850_bus[bus].state ++;
			j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].bit_ptr = 0;
			j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].byte_ptr = 0;
			j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].bytes = 0;
		}
	}
	else if(j1850_bus[bus].state == 2) {
		//Receive data bits
		if(j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].byte_ptr < 12 && delta > RX_SHORT_MIN && delta < RX_LONG_MAX) {
			//Setup EOD interrupt
			j1850_set_delay(tmr_cnt, bus, RX_EOD_MIN);
			
			j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].buf[j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].byte_ptr] <<= 1;
			if((pin && delta > RX_LONG_MIN) || (!pin && delta < RX_SHORT_MAX)) {
				//Passive 1
				j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].buf[j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].byte_ptr] |= 1;
			}
			j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].bit_ptr ++;
			
			if(j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].bit_ptr == 8) {
				j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].bit_ptr = 0;
				j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].byte_ptr ++;
				j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_end].bytes ++;
			}
		}
		else {
			//We've started the 13th byte or the pulse was too short/long, something went wrong
			j1850_stop_tmr(bus);
			j1850_bus[bus].state = 0;
		}
	}
	else if(j1850_bus[bus].state == 10) {
		//Trying to initiate a transmission, do nothing
	}
	else if(j1850_bus[bus].state == 11) {
		//Pin changed while we were waiting for IFS, reset timer
		j1850_set_delay(tmr_cnt, bus, TX_IFS);
	}
	else {
		//Reset
		j1850_bus[bus].state = 0;
	}
}

inline void j1850_service_tmr(uint8_t *tmr_cnt, uint8_t bus) {
	if(j1850_bus[bus].state == 2) {
		//Received EOD
		j1850_stop_tmr(bus);
		j1850_bus[bus].new_msg = 1;
		j1850_bus[bus].rx_msg_end++;
		if(j1850_bus[bus].rx_msg_end == J1850_MSG_BUF_SIZE) j1850_bus[bus].rx_msg_end = 0;
		j1850_bus[bus].state = 0;
	}
	else if(j1850_bus[bus].state == 10) {
		//Try to send something
		j1850_bus[bus].state = 11;
		j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].bit_ptr = 0;
		j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].byte_ptr = 0xFF;
		j1850_set_pin(bus, 0);
		j1850_set_delay(tmr_cnt, bus, TX_IFS);
	}
	else if(j1850_bus[bus].state == 11) {
		//Waiting for IFS
		uint8_t j1850_pin = *j1850_bus[bus].pin_reg & j1850_bus[bus].pin_msk;
		uint8_t delay;
		
		if(j1850_pin) {
			//Bus isn't passive
			delay = TX_IFS;
		}
		else {
			//Successfully waited for IFS
			j1850_bus[bus].state = 12;
			*j1850_bus[bus].pcint_reg &= ~j1850_bus[bus].pcint_msk;
			j1850_set_pin(bus, 1);
			delay = TX_SOF;
		}
		
		j1850_set_delay(tmr_cnt, bus, delay);
	}
	else if(j1850_bus[bus].state == 12) {
		//Sending bits
		uint8_t pin = *j1850_bus[bus].pin_reg & j1850_bus[bus].pin_msk;
		j1850_set_pin(bus, !pin);
		
		if(j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].bit_ptr) j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].bit_ptr --;
		else if(j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].bytes) {
			j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].bytes --;
			j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].byte_ptr ++;
			j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].bit_ptr = 7;
		}
		else {
			//Done sending bits
			j1850_bus[bus].state = 0;
			
			j1850_bus[bus].tx_msg_start++;
			if(j1850_bus[bus].tx_msg_start == J1850_MSG_BUF_SIZE) j1850_bus[bus].tx_msg_start = 0;
			
			*j1850_bus[bus].pcint_reg |= j1850_bus[bus].pcint_msk;
			j1850_stop_tmr(bus);
			return;
		}
		
		uint8_t delay;
		
		if(!(j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].buf[j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].byte_ptr] & 0x80) != !pin) delay = TX_SHORT;
		else delay = TX_LONG;
		
		j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].buf[j1850_bus[bus].tx_buf[j1850_bus[bus].tx_msg_start].byte_ptr] <<= 1;
		j1850_set_delay(tmr_cnt, bus, delay);
	}
	else {
		//Not sure why we're here
		j1850_stop_tmr(bus);
	}
}

/*
 * Service the pin change ISR and direct it to the proper bus
 */
ISR(PCINT2_vect) {
	PORTB ^= (1<<PORTB1);
	/*
//    uint8_t tmr_cnt = TCNT2;
    
    uint8_t bus = 0;
    //for(bus = 0; bus < 2; bus++) {
		/*
		uint8_t changed = (*j1850_bus[bus].pin_reg ^ j1850_bus[bus].last_pin) & j1850_bus[bus].pin_msk;
		j1850_bus[bus].last_pin = *j1850_bus[bus].pin_reg;
		
		if(changed && *j1850_bus[bus].pcint_reg & j1850_bus[bus].pcint_msk) j1850_service_pin(&tmr_cnt, bus);
		*
		uint8_t otherbus = 1;
		uint8_t port = *j1850_bus[bus].port_reg & j1850_bus[bus].port_msk;
		uint8_t pin = *j1850_bus[bus].pin_reg & j1850_bus[bus].pin_msk;
		if(bus) otherbus = 0;
		uint8_t otherpin = *j1850_bus[otherbus].pin_reg & j1850_bus[otherbus].pin_msk;
		
		if(pin && !port) j1850_set_pin(otherbus, 1);
		else j1850_set_pin(otherbus, 0);
	//}
	*/
}

/*
 * J1850 channel timer compare interrupts
 */
ISR(TIMER2_COMPA_vect) {
	uint8_t tmr_cnt = TCNT2;
	j1850_service_tmr(&tmr_cnt, 0);
}
ISR(TIMER2_COMPB_vect) {
	uint8_t tmr_cnt = TCNT2;
	j1850_service_tmr(&tmr_cnt, 1);
}

/*
 * Starts the interrupt system sending out the next message in the buffer
 */
void j1850_send_packet(uint8_t bus) {
	cli();
	j1850_bus[bus].state = 10;
	if(bus) {
		OCR2B = TCNT2 + 1;
		TIFR2 |= (1<<OCF2B);
		TIMSK2 |= (1<<OCIE2B);
	}
	else {
		OCR2A = TCNT2 + 1;
		TIFR2 |= (1<<OCF2A);
		TIMSK2 |= (1<<OCIE2A);
	}
	sei();
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
	j1850_wire_mode = 1; 
	
    j1850_bus[0].port_reg = &PORTB;
    j1850_bus[0].ddrport_reg = &DDRB;
    j1850_bus[0].port_msk = (1<<PORTB1);
    j1850_bus[0].pin_reg = &PIND;
    j1850_bus[0].ddrpin_reg = &DDRD;
    j1850_bus[0].pin_msk = (1<<PORTD2);
    j1850_bus[0].pcint_reg = &PCMSK2;
    j1850_bus[0].pcint_msk = (1<<PCINT18);
    
    j1850_bus[1].port_reg = &PORTD;
    j1850_bus[1].ddrport_reg = &DDRD;
    j1850_bus[1].port_msk = (1<<PORTD6);
    j1850_bus[1].pin_reg = &PIND;
    j1850_bus[1].ddrpin_reg = &DDRD;
    j1850_bus[1].pin_msk = (1<<PORTD3);
    j1850_bus[1].pcint_reg = &PCMSK2;
    j1850_bus[1].pcint_msk = (1<<PCINT19);
    
    uint8_t bus;
    for(bus = 0; bus < J1850_BUSSES; bus++) {
		*j1850_bus[bus].ddrport_reg |= j1850_bus[bus].port_msk;
		*j1850_bus[bus].ddrpin_reg &= ~j1850_bus[bus].port_msk;
		*j1850_bus[bus].pcint_reg |= j1850_bus[bus].pcint_msk;
	}
    
    *j1850_bus[0].pcint_reg &= ~j1850_bus[0].pcint_msk;
    
    TCCR2A = 0;
    //1/64 prescaler
    TCCR2B = (1<<CS21) | (1<<CS20);
    
    //Enable PCI
    PCICR = (1<<PCIE2);
}
