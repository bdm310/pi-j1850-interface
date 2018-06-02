/*
 * j1850.c - J1850 interface
 */

#include "j1850.h"

inline void j1850_set_pin(uint8_t bus, uint8_t state) {
	if(bus) {
		if(state) PORTD |= (1<<PORTD6);
		else PORTD &= ~(1<<PORTD6);
	}
	else {
		if(state) PORTB |= (1<<PORTB1);
		else PORTB &= ~(1<<PORTB1);
	}
}
/*
inline uint8_t j1850_get_pin(j1850_bus_t *bus) {
	if(bus->pin_reg & bus->pin_msk) return 1;
	return 0;
}
*/
inline uint8_t j1850_get_pin(uint8_t bus) {
	if(bus == 0) {
		if(PORTB & (1<<PORTB1)) return 1;
	}
	else {
		if(PORTD & (1<<PORTD6)) return 1;
	}
	return 0;
}

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

void j1850_service_pin(uint8_t *tmr_cnt, uint8_t bus, uint8_t *pin) {
	uint8_t delta = *tmr_cnt - j1850_ltmr[bus];
	j1850_ltmr[bus] = *tmr_cnt;
	
	if(j1850_state[bus] == 0 && *pin) {
		//Transistion away from idle
		j1850_state[bus] ++;
	}
	else if(j1850_state[bus] == 1 && !*pin) {
		//Check for SOF
		if(delta > RX_SOF_MAX || delta < RX_SOF_MIN) {
			j1850_state[bus] = 0;
		}
		else {
			j1850_state[bus] ++;
			j1850_rx_bit[bus] = 0;
			j1850_rx_bytes[bus] = 0;
		}
	}
	else if(j1850_state[bus] == 2) {
		//Receive data bits
		if(j1850_rx_bytes[bus] < 12 && delta > RX_SHORT_MIN && delta < RX_LONG_MAX) {
			//Setup EOD interrupt
			j1850_set_delay(tmr_cnt, bus, RX_EOD_MIN);
			
			j1850_rx_byte[bus] <<= 1;
			if(*pin && delta > RX_LONG_MIN) {
				//Passive 1
				j1850_rx_byte[bus] |= 1;
			}
			if(!*pin && delta < RX_SHORT_MAX) {
				//Active 1
				j1850_rx_byte[bus] |= 1;
			}
			j1850_rx_bit[bus] ++;
			
			if(j1850_rx_bit[bus] == 8) {
				j1850_rx_bit[bus] = 0;
				j1850_rx_buf[bus][j1850_rx_bytes[bus]] = j1850_rx_byte[bus];
				j1850_rx_bytes[bus] ++;
			}
		}
		else {
			//We've started the 13th byte or the pulse was too short/long, something went wrong
			if(bus == 0) TIMSK2 &= ~(1<<OCIE2A);
			else TIMSK2 &= ~(1<<OCIE2B);
			j1850_state[bus] = 0;
		}
	}
	else if(j1850_state[bus] == 10) {
		//Trying to initiate a transmission, do nothing
	}
	else if(j1850_state[bus] == 11) {
		//Pin changed while we were waiting for IFS, reset timer
		j1850_set_delay(tmr_cnt, bus, TX_IFS);
	}
	else {
		//Reset
		j1850_state[bus] = 0;
	}
}

void j1850_service_tmr(uint8_t *tmr_cnt, uint8_t bus) {
	if(j1850_state[bus] == 2) {
		//Received EOD
		if(bus == 0) TIMSK2 &= ~(1<<OCIE2A);
		else TIMSK2 &= ~(1<<OCIE2B);
		
		j1850_new_msg[bus] = 1;
			
		j1850_state[bus] = 0;
	}
	else if(j1850_state[bus] == 10) {
		//Try to send something
		j1850_state[bus] = 11;
		
		j1850_tx_bit[bus] = 0;
		j1850_tx_byte[bus] = 0xFF;
		
		j1850_set_pin(bus, 0);
		
		j1850_set_delay(tmr_cnt, bus, TX_IFS);
	}
	else if(j1850_state[bus] == 11) {
		//Waiting for IFS
		uint8_t j1850_pin;
		uint8_t delay;
		
		if(bus == 0) j1850_pin = PIND & (1<<PIND2);
		else j1850_pin = PIND & (1<<PIND3);
		
		if(j1850_pin) {
			//Bus isn't passive
			delay = TX_IFS;
		}
		else {
			//Successfully waited for IFS
			j1850_state[bus] = 12;
			
			if(bus) PCMSK2 &= ~(1<<PCINT19);
			else PCMSK2 &= ~(1<<PCINT18);
		
			j1850_set_pin(bus, 1);
			delay = TX_SOF;
		}
		
		j1850_set_delay(tmr_cnt, bus, delay);
	}
	else if(j1850_state[bus] == 12) {
		//Sending bits
		uint8_t pin = j1850_get_pin(bus);
		j1850_set_pin(bus, !pin);
		
		if(j1850_tx_bit[bus]) j1850_tx_bit[bus] --;
		else if(j1850_tx_bytes[bus]) {
			j1850_tx_bytes[bus] --;
			j1850_tx_byte[bus] ++;
			j1850_tx_bit[bus] = 7;
		}
		else {
			//Done sending bits
			j1850_state[bus] = 0;
			
			if(bus) PCMSK2 &= ~(1<<PCINT19);
			else PCMSK2 &= ~(1<<PCINT18);
			
			if(bus == 0) TIMSK2 &= ~(1<<OCIE2A);
			else TIMSK2 &= ~(1<<OCIE2B);
			return;
		}
		
		uint8_t delay;
		
		if(!(j1850_tx_buf[bus][j1850_tx_byte[bus]] & 0x80) != !pin) delay = TX_SHORT;
		else delay = TX_LONG;
		
		j1850_tx_buf[bus][j1850_tx_byte[bus]] <<= 1;
		
		j1850_set_delay(tmr_cnt, bus, delay);
	}
	else {
		//Not sure why we're here
		if(bus == 0) TIMSK2 &= ~(1<<OCIE2A);
		else TIMSK2 &= ~(1<<OCIE2B);
	}
}

ISR(PCINT2_vect) {
    uint8_t tmr_cnt = TCNT2;
    /*
    uint8_t i;
    for(i=0; i<2; i++) {
		uint8_t changed = (*j1850_bus[i].pin_reg ^ j1850_bus[i].last_pin) & j1850_bus[i].pin_msk;
		j1850_bus[i].last_pin = *j1850_bus[i].pin_reg;
		
		if(changed) j1850_service_pin(&timer_cnt, i);
	}
    */
    uint8_t changed = j1850_last_pin ^ PIND;
    j1850_last_pin = PIND;
    
    if(changed & (1<<PIND2)) {
        //J1850_IN_FLT
        uint8_t j1850_pin = j1850_last_pin & (1<<PIND2);
        j1850_service_pin(&tmr_cnt, 0, &j1850_pin);
    }
    else if(changed & (1<<PIND3)) {
        //J1850_OUT_FLT
        uint8_t j1850_pin = j1850_last_pin & (1<<PIND3);
        j1850_service_pin(&tmr_cnt, 1, &j1850_pin);
    }
}

ISR(TIMER2_COMPA_vect) {
	uint8_t tmr_cnt = TCNT2;
	j1850_service_tmr(&tmr_cnt, 0);
}

ISR(TIMER2_COMPB_vect) {
	uint8_t tmr_cnt = TCNT2;
	j1850_service_tmr(&tmr_cnt, 1);
}

void j1850_send_packet(uint8_t bus) {
	cli();
	j1850_state[bus] = 10;
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

void j1850_init(void) {
    //J1850_OUT_CMD
    DDRD |= (1<<PORTD6);
    //J1850_OUT_FLT
    DDRD &= ~(1<<PORTD3);
    //J1850_IN_CMD
    DDRB |= (1<<PORTB1);
    //J1850_IN_FLT
    DDRD &= ~(1<<PORTD2);
    
    uint8_t i;
    for(i = 0; i < 1; i++) {
		j1850_tx_bit[i] = 0;
		j1850_tx_bytes[i] = 0;
		j1850_new_msg[i] = 0;
	}
    
    TCCR2A = 0;
    //1/64 prescaler
    TCCR2B = (1<<CS21) | (1<<CS20);
    
    PCMSK2 = (1<<PCINT18) | (1<<PCINT19);
    PCICR = (1<<PCIE2);
}
