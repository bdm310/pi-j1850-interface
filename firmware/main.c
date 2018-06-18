/*
 * main.c - Pi J1850 Interface
 */
 
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "spi.h"
#include "main.h"
#include "j1850.h"


ISR(TIMER0_COMPA_vect) {
    timer_10ms ++;
    cnt_10ms ++;
	
    if(cnt_10ms == 100) {
		cnt_10ms = 0;
		timer_1s ++;
	}
}


ISR(ADC_vect) {
    uint8_t res = ADCH;
	
    //Are we on channel 1?
    if(ADMUX & (1<<MUX0)) {
        sw1_adc = res;
        sw_new = 1;
        ADMUX &= ~(1<<MUX0);
    }
    else {
        sw0_adc = res;

        //Move to channel 1 and start the conversion
        ADMUX |= (1<<MUX0);
        ADCSRA |= (1<<ADSC);
    }

}


void timer_10ms_init(void) {
    //Select CTC mode
    TCCR0A = 0b00000010;
    //Select x1024 prescalar for TC0
    TCCR0B = 0b00000101;
    //Set a 10ms timeout 78
    OCR0A = 78;
    //Enable OC0A interrupt
    TIMSK0 = 0b00000010;
}


void io_init(void) {
    DDRD |= (1<<PORTD4);
    PORTD |= (1<<PORTD4);
}

void ad_init(void) {
    //Set result as left justified
    ADMUX |= (1<<ADLAR);
    //Auto trigger on TC0 match
    ADCSRB |= (1<<ADTS1) | (1<<ADTS0);
    //Disable digital on AN0, AN1
    DIDR0 |= (1<<ADC1D) | (1<<ADC0D);
    //Enable, auto trigger, interrupt enabled, 1/16 clock rate
    ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS2);
}

void read_sw_state(void) {
    uint8_t sw_state_new = 0;

    if(sw0_adc > 213) sw_state_new |= 0x01;
    else if(sw0_adc > 169) sw_state_new |= 0x02;
    else if(sw0_adc > 119) sw_state_new |= 0x03;
    else if(sw0_adc > 75) sw_state_new |= 0x04;
    else if(sw0_adc > 27) sw_state_new |= 0x05;

    if(sw1_adc > 213) sw_state_new |= 0x10;
    else if(sw1_adc > 169) sw_state_new |= 0x20;
    else if(sw1_adc > 119) sw_state_new |= 0x30;
    else if(sw1_adc > 75) sw_state_new |= 0x40;
    else if(sw1_adc > 27) sw_state_new |= 0x50;

    sw_state = sw_state_new;
}

int main(void) {
	uint8_t waiting = 1;
	uint8_t shutdown_tmr = 0;
	uint8_t cd_status = 0;
	uint8_t bus;
	
	//Reset and turn off WDT
	wdt_reset();
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 0x00;
	
    timer_10ms_init();
    ad_init();
    io_init();
    
    j1850_init();
    j1850_listen_bytes = 1;
    j1850_listen_headers[0] = 0x8D;
    
    spi_active = 0;
    spi_init_slave();                             //Initialize slave SPI
	
    sei();
    
    for (;;)
    {
		if(waiting && timer_1s > 40) waiting = 0;
		if(!waiting) {
			if(PINC & (1<<PINC5)) {
				PORTD |= (1<<PORTD4);
				shutdown_tmr = timer_10ms;
			}
			else {
				if((uint8_t)(timer_10ms - shutdown_tmr) > 20) {
					PORTD &= ~(1<<PORTD4);
				}
			}
		}
		cli();
		if((uint8_t)(timer_10ms - last_spi_byte_tmr) > 50) {
			spi_status = 0;
		}
		sei();
		
		cli();
		for(bus=0; bus<2; bus++) {
			if((j1850_bus[bus].tx_msg_start != j1850_bus[bus].tx_msg_end) && j1850_bus[bus].state == 0) {
				j1850_send_packet(bus);
			}
		}
		sei();
		
		for(bus=0; bus<2; bus++) {
			volatile j1850_bus_t *j1850 = &j1850_bus[bus];
			if(j1850->rx_msg_start != j1850->rx_msg_end) {
				volatile j1850_msg_buf_t *msg = &j1850->rx_buf[j1850->rx_msg_start];
				
				uint8_t match = 0;
				uint8_t i;
				if(j1850_listen_bytes) {
					for(i=0; i<j1850_listen_bytes; i++) {
						if(msg->buf[0] == j1850_listen_headers[i]) {
							match = 1;
							break;
						}
					}
				}
				else match = 1;
				
				if(match) {
					volatile j1850_msg_buf_t *rxmsg;
					if(bus) rxmsg = &bus1_rx[bus1_rx_end];
					else rxmsg = &bus0_rx[bus0_rx_end];
					
					//If the daemon isn't up yet, pretend to be a satellite module
					if(bus == 0 && !spi_active) {
						if(msg->buf[0] == 0x8D && msg->buf[1] == 0x0F) {
							if(msg->buf[2] == 0x26) cd_status = 2;
							else cd_status = 1;
						}
					}
					
					*rxmsg = *msg;
					
					cli();
					if(bus) {
						bus1_rx_end = bus1_rx_end + 1;;
						if(bus1_rx_end == J1850_MSG_BUF_SIZE) bus1_rx_end = 0;
					}
					else {
						bus0_rx_end = bus0_rx_end + 1;;
						if(bus0_rx_end == J1850_MSG_BUF_SIZE) bus0_rx_end = 0;
					}
					sei();
				}
				
				j1850->rx_msg_start ++;
				if(j1850->rx_msg_start == J1850_MSG_BUF_SIZE) j1850->rx_msg_start = 0;
			}
		}
        
		if(cd_status) {
			switch( cd_status ){
				case 1:
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[0] = 0x8D;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[1] = 0x22;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[2] = 0x10;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[3] = 0x00;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[4] = 0x01;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[5] = j1850_crc((uint8_t *)&j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[0], 5);
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].bytes = 6;
					cli();
					j1850_bus[0].tx_msg_end++;
					if(j1850_bus[0].tx_msg_end == J1850_MSG_BUF_SIZE) j1850_bus[0].tx_msg_end = 0;
					sei();
					cd_status = 0;
					break;
				case 2:
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[0] = 0x8D;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[1] = 0x22;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[2] = 0x11;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[3] = 0x01;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[4] = 0x01;
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[5] = j1850_crc((uint8_t *)&j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].buf[0], 5);
					j1850_bus[0].tx_buf[j1850_bus[0].tx_msg_end].bytes = 6;
					cli();
					j1850_bus[0].tx_msg_end++;
					if(j1850_bus[0].tx_msg_end == J1850_MSG_BUF_SIZE) j1850_bus[0].tx_msg_end = 0;
					sei();
					cd_status = 0;
					break;
			}
		}
		
		if(sw_new) {
			sw_new = 0;
			read_sw_state();
		}
    }
    return (0);
}
