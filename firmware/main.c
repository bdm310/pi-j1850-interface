/*
 * main.c - Pi J1850 Interface
 */
 
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "spi.h"
#include "main.h"
#include "j1850.h"

FUSES = 
{
    .low = LFUSE_DEFAULT | FUSE_CKDIV8,
    .high = HFUSE_DEFAULT,
    .extended = EFUSE_DEFAULT,
};

ISR(TIMER0_COMPA_vect) {
    timer_10ms ++;
    cnt_10ms ++;

    if(cnt_10ms == 100) {
		cnt_10ms = 0;
		timer_1s ++;
	}
}

ISR(ADC_vect) {
    uint8_t res_l = ADCL;
    uint8_t res_h = ADCH;
	
    //Are we on channel 0?
    if(!(ADMUX & (1<<MUX0))) {
        sw0_adc = res_h;

        //Move to channel 1 and start the conversion
        ADMUX |= (1<<MUX0);
        ADCSRA |= (1<<ADSC);
    }
    else {
        sw1_adc = res_h;
        sw_new = 1;
        ADMUX &= ~(1<<MUX0);
    }

}

void timer_10ms_init(void) {
    //Select CTC mode
    TCCR0A = 0b00000010;
    //Select x1024 prescalar for TC0
    TCCR0B = 0b00000101;
    //Set a 10ms timeout
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
	uint8_t byte0;
	uint8_t byte1;
	uint8_t byte2;
	uint8_t cd_status = 0;
	uint8_t lsw_state = 0;
	uint8_t waiting = 1;
	uint8_t shutdown_tmr = 0;
	
    sw_state = 0;
    sw_new = 0;

//    timer_10ms_init();
    io_init();
//    ad_init();
    j1850_init();
    
    spi_init_slave();                             //Initialize slave SPI

    sei();

    /* loop forever, the interrupts are doing the rest */
    for (;;)
    {
		if(waiting && timer_1s > 40) waiting = 0;
		if(!waiting) {
			if(PINC & (1<<PINC5)) shutdown_tmr = timer_10ms;
			else {
				if(timer_10ms - shutdown_tmr > 20) {
					PORTD &= ~(1<<PORTD4);
					while(1);
				}
			}
		}
		cli();
		if(timer_10ms - last_spi_byte_tmr > 50) {
			spi_status = 0;
		}
		sei();
		
		uint8_t bus;
		for(bus=0; bus<J1850_BUSSES; bus++) {
			/*
			if(j1850_bus[bus].rx_msg_end != j1850_bus[bus].rx_msg_start) {
				uint8_t otherbus = 1;
				if(bus) otherbus = 0;
				
				uint8_t i;
				for(i = 0; i < 12; i++) {
					j1850_bus[otherbus].tx_buf[j1850_bus[otherbus].tx_msg_end].buf[i] = j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_start].buf[i];
				}
				
				j1850_bus[otherbus].tx_buf[j1850_bus[otherbus].tx_msg_end].bytes = j1850_bus[bus].rx_buf[j1850_bus[bus].rx_msg_start].bytes;
				j1850_bus[otherbus].tx_msg_end++;
				if(j1850_bus[otherbus].tx_msg_end == J1850_MSG_BUF_SIZE) j1850_bus[otherbus].tx_msg_end = 0;
				j1850_bus[bus].rx_msg_start++;
				if(j1850_bus[bus].rx_msg_start == J1850_MSG_BUF_SIZE) j1850_bus[bus].rx_msg_start = 0;
			}
			if(j1850_bus[bus].tx_msg_end != j1850_bus[bus].tx_msg_start && j1850_bus[bus].state == 0) {
				j1850_send_packet(bus);
			}
			*/
			/*
			if(j1850_bus[i].new_msg) {
				uint8_t otheri;
				if(i) otheri = 0;
				else otheri = 1;
				
				j1850_bus[i].new_msg = 0;
				
				uint8_t byte;
				for(byte = 0; byte < j1850_bus[i].rx_buf.bytes; byte++) {
					cli();
					ring_buf_push((ring_buf_t *)&rx_buf[i], j1850_bus[i].rx_buf.buf[byte]);
					sei();
					j1850_bus[otheri].tx_buf.buf[byte] =  j1850_bus[i].rx_buf.buf[byte];
				}
				j1850_bus[otheri].tx_buf.bytes = j1850_bus[i].rx_buf.bytes;
				j1850_send_packet(otheri);
			
				byte0 = j1850_bus[i].rx_buf.buf[0];
				byte1 = j1850_bus[i].rx_buf.buf[1];
				byte2 = j1850_bus[i].rx_buf.buf[2];
				if(byte0 == 0x08D && byte1 == 0x0F && cd_status == 0) {
					if(byte2 == 0x21) cd_status = 2;
					else if(byte2 == 0x24) cd_status = 5;
					else cd_status = 1;
				}
			}
			*/
		}
		
		/*
		if(sw_new) {
            sw_new = 0;
            read_sw_state();
            
            if(lsw_state ^ sw_state) {
				lsw_state = sw_state;
				switch(sw_state) {
					case 0x00:
						cd_status = 100;
						break;
					case 0x01:
						cd_status = 101;
						break;
					case 0x02:
						cd_status = 102;
						break;
					case 0x03:
						cd_status = 103;
						break;
					case 0x04:
						cd_status = 104;
						break;
					case 0x05:
						cd_status = 105;
						break;
				}
			}
        }
		if(cd_status) {
			switch( cd_status ){
				case 1:
					j1850_bus[1].tx_buf.buf[0] = 0x8D;
					j1850_bus[1].tx_buf.buf[1] = 0x93;
					j1850_bus[1].tx_buf.buf[2] = 0x01;
					j1850_bus[1].tx_buf.buf[3] = 0x01;
					j1850_bus[1].tx_buf.buf[4] = 0x80;
					j1850_bus[1].tx_buf.buf[5] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 4);
					j1850_bus[1].tx_buf.bytes = 6;
					j1850_send_packet(1);
					cd_status = 0;
					break;
				case 2:
					j1850_bus[1].tx_buf.buf[0] = 0x8D;
					j1850_bus[1].tx_buf.buf[1] = 0x92;
					j1850_bus[1].tx_buf.buf[2] = 0xC0;
					j1850_bus[1].tx_buf.buf[3] = 0x00;
					j1850_bus[1].tx_buf.buf[4] = 0x00;
					j1850_bus[1].tx_buf.buf[5] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 5);
					j1850_bus[1].tx_buf.bytes = 6;
					j1850_send_packet(1);
					cd_status = 6;
					break;
				case 3:
					j1850_bus[1].tx_buf.buf[0] = 0x8D;
					j1850_bus[1].tx_buf.buf[1] = 0x92;
					j1850_bus[1].tx_buf.buf[2] = 0xE1;
					j1850_bus[1].tx_buf.buf[3] = 0x01;
					j1850_bus[1].tx_buf.buf[4] = 0x03;
					j1850_bus[1].tx_buf.buf[5] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 5);
					j1850_bus[1].tx_buf.bytes = 6;
					j1850_send_packet(1);
					cd_status = 7;
					break;
				case 4:
					j1850_bus[1].tx_buf.buf[0] = 0x8D;
					j1850_bus[1].tx_buf.buf[1] = 0x93;
					j1850_bus[1].tx_buf.buf[2] = 0x01;
					j1850_bus[1].tx_buf.buf[3] = 0x01;
					j1850_bus[1].tx_buf.buf[4] = 0x80;
					j1850_bus[1].tx_buf.buf[5] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 5);
					j1850_bus[1].tx_buf.bytes = 6;
					j1850_send_packet(1);
					cd_status = 0;
					break;
				case 5:
					j1850_bus[1].tx_buf.buf[0] = 0x8D;
					j1850_bus[1].tx_buf.buf[1] = 0x94;
					j1850_bus[1].tx_buf.buf[2] = 0x00;
					j1850_bus[1].tx_buf.buf[3] = 0x00;
					j1850_bus[1].tx_buf.buf[4] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 4);
					j1850_bus[1].tx_buf.bytes = 5;
					j1850_send_packet(1);
					cd_status = 0;
					break;
				case 6:
					if(j1850_bus[1].state == 0) cd_status = 3;
					break;
				case 7:
					if(j1850_bus[1].state == 0) cd_status = 4;
					break;
				case 100:
					j1850_bus[1].tx_buf.buf[0] = 0x3D;
					j1850_bus[1].tx_buf.buf[1] = 0x11;
					j1850_bus[1].tx_buf.buf[2] = 0x00;
					j1850_bus[1].tx_buf.buf[3] = 0x00;
					j1850_bus[1].tx_buf.buf[4] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 4);
					j1850_bus[1].tx_buf.bytes = 5;
					j1850_send_packet(1);
					cd_status = 120;
					break;
				case 101:
					j1850_bus[1].tx_buf.buf[0] = 0x3D;
					j1850_bus[1].tx_buf.buf[1] = 0x11;
					j1850_bus[1].tx_buf.buf[2] = 0x00;
					j1850_bus[1].tx_buf.buf[3] = 0x02;
					j1850_bus[1].tx_buf.buf[4] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 4);
					j1850_bus[1].tx_buf.bytes = 5;
					j1850_send_packet(1);
					cd_status = 120;
					break;
				case 102:
					j1850_bus[1].tx_buf.buf[0] = 0x3D;
					j1850_bus[1].tx_buf.buf[1] = 0x11;
					j1850_bus[1].tx_buf.buf[2] = 0x10;
					j1850_bus[1].tx_buf.buf[3] = 0x00;
					j1850_bus[1].tx_buf.buf[4] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 4);
					j1850_bus[1].tx_buf.bytes = 5;
					j1850_send_packet(1);
					cd_status = 120;
					break;
				case 103:
					j1850_bus[1].tx_buf.buf[0] = 0x3D;
					j1850_bus[1].tx_buf.buf[1] = 0x11;
					j1850_bus[1].tx_buf.buf[2] = 0x02;
					j1850_bus[1].tx_buf.buf[3] = 0x00;
					j1850_bus[1].tx_buf.buf[4] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 4);
					j1850_bus[1].tx_buf.bytes = 5;
					j1850_send_packet(1);
					cd_status = 120;
					break;
				case 104:
					j1850_bus[1].tx_buf.buf[0] = 0x3D;
					j1850_bus[1].tx_buf.buf[1] = 0x11;
					j1850_bus[1].tx_buf.buf[2] = 0x04;
					j1850_bus[1].tx_buf.buf[3] = 0x00;
					j1850_bus[1].tx_buf.buf[4] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 4);
					j1850_bus[1].tx_buf.bytes = 5;
					j1850_send_packet(1);
					cd_status = 120;
					break;
				case 105:
					j1850_bus[1].tx_buf.buf[0] = 0x3D;
					j1850_bus[1].tx_buf.buf[1] = 0x11;
					j1850_bus[1].tx_buf.buf[2] = 0x10;
					j1850_bus[1].tx_buf.buf[3] = 0x00;
					j1850_bus[1].tx_buf.buf[4] = j1850_crc((uint8_t *)&j1850_bus[1].tx_buf.buf[0], 4);
					j1850_bus[1].tx_buf.bytes = 5;
					j1850_send_packet(1);
					cd_status = 120;
					break;
				case 120:
					if(j1850_bus[1].state == 0) cd_status = 0;
					break;
			}
		}
		*/
    }
    return (0);
}
