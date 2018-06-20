/*
 * main.c - Pi J1850 Interface
 */

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "main.h"
#include "spi.h"
#include "j1850.h"

static volatile uint8_t sw_adc[2];
static volatile uint8_t sw_new;

static uint8_t last_sw_state;
static uint8_t last_pwr_state;

ISR(TIMER0_COMPA_vect) {
    static uint8_t cnt_10ms;
    
    tmr_10ms ++;
    cnt_10ms ++;
    
    if(cnt_10ms == 100) {
        cnt_10ms = 0;
        tmr_1s ++;
    }
}

ISR(ADC_vect) {
    //Are we on channel 1?
    if(ADMUX & (1<<MUX0)) {
        sw_adc[1] = ADCH;
        sw_new = 1;
    }
    else sw_adc[0] = ADCH;
        
    //Move channels and start the conversion
    ADMUX = ADMUX ^ (1<<MUX0);
}

static void tmrs_init(void) {
    tmr_10ms = 0;
    tmr_1s = 0;
    
    //Select CTC mode
    TCCR0A = 0b00000010;
    //Set a 10ms timeout with a x1024 prescaler
    TCCR0B = 0b00000101;
    OCR0A = 78;
    TIMSK0 |= (1<<OCIE0A);
}

static void io_init(void) {
    //Power hold output
    POWER_HOLD_DDR |= POWER_HOLD_MSK;
    POWER_HOLD_PORT |= POWER_HOLD_MSK;
    
    DDRC |= (1<<PORTC2) | (1<<PORTC3);
}

static void ad_init(void) {
    //Set result as left justified
    ADMUX |= (1<<ADLAR);
    //Auto trigger on TC0 match
    ADCSRB |= (1<<ADTS1) | (1<<ADTS0);
    //Disable digital on AN0, AN1
    DIDR0 |= (1<<ADC1D) | (1<<ADC0D);
    //Enable, auto trigger, interrupt enabled, 1/16 clock rate
    ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS2);
}

static void read_input_state(void) {
    uint8_t this_sw_state = 0;
    uint8_t this_pwr_state = 0;
    
    uint8_t sw;
    for(sw=0; sw<2; sw++) {
        uint8_t rot = 0;
        if(sw) rot = 4;
        
        if(sw_adc[sw] > SW_THRESH_4) this_sw_state |= 0x01<<rot;
        else if(sw_adc[sw] > SW_THRESH_3) this_sw_state |= 0x02<<rot;
        else if(sw_adc[sw] > SW_THRESH_2) this_sw_state |= 0x03<<rot;
        else if(sw_adc[sw] > SW_THRESH_1) this_sw_state |= 0x04<<rot;
        else if(sw_adc[sw] > SW_THRESH_0) this_sw_state |= 0x05<<rot;
    }
    
    //Do a little debouncing
    if(this_sw_state == last_sw_state) sw_state = this_sw_state;
    last_sw_state = this_sw_state;
    
    if(ACC_REG & ACC_MSK) this_pwr_state |= 0x01;
    if(PI_REG & PI_MSK) this_pwr_state |= 0x02;
    
    //Do a little debouncing
    if(this_pwr_state == last_pwr_state) pwr_state = this_pwr_state;
    last_pwr_state = this_pwr_state;
}

int main(void) {
    //Reset and turn off WDT
    wdt_reset();
    MCUSR &= ~(1<<WDRF);
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    WDTCSR = 0x00;
    
    uint8_t waiting = 1;
    uint8_t shutdown_tmr = 0;

    //Timers
    tmrs_init();
    
    //Other peripherals
    ad_init();
    io_init();
    spi_init_slave();
    
    j1850_init();
    j1850_listen_bytes = 1;
    j1850_listen_headers[0] = 0x8D;
    
    sei();
    
    for (;;)
    {
        //Do power handling
        if(waiting && tmr_1s > 40) waiting = 0;
		if(!waiting) {
			if(PI_REG & PI_MSK) {
				POWER_HOLD_PORT |= POWER_HOLD_MSK;
				shutdown_tmr = tmr_10ms;
			}
			else {
				if((uint8_t)(tmr_10ms - shutdown_tmr) > 20) {
					POWER_HOLD_PORT &= ~POWER_HOLD_PORT;
				}
			}
        }

        //Check for AD results, convert to switch states, check other inputs
        if(sw_new) {
            sw_new = 0;
            read_input_state();
        }
        
        //Do SPI
        spi_process(tmr_10ms);
        
        //Do J1850
        j1850_process();
        
        //If the pi hasn't started yet, do the minimum to pretend to be a satellite module
        //and remove messages from the buffer as they come in
        if(!spi_active) {
            j1850_msg_buf_t *start = (j1850_msg_buf_t *)j1850_bus[0].rx_msg_start;
            cli();
            j1850_msg_buf_t *end = (j1850_msg_buf_t *)j1850_bus[0].rx_msg_end;
            sei();
        
            if(start != end) {
                if(start->buf[0] == 0x8D && start->buf[1] == 0x0F) {
                    j1850_msg_buf_t *end = (j1850_msg_buf_t *)j1850_bus[0].tx_msg_end;
                    if(start->buf[2] == 0x26) {
                        end->buf[0] = 0x8D;
                        end->buf[1] = 0x22;
                        end->buf[2] = 0x11;
                        end->buf[3] = 0x01;
                        end->buf[4] = 0x01;
                        end->buf[5] = j1850_crc(end->buf, 5);
                        end->bytes = 6;
                    }
                    else {
                        end->buf[0] = 0x8D;
                        end->buf[1] = 0x22;
                        end->buf[2] = 0x10;
                        end->buf[3] = 0x00;
                        end->buf[4] = 0x01;
                        end->buf[5] = j1850_crc(end->buf, 5);
                        end->bytes = 6;
                    }
                    
                    end ++;
                    if(end == &j1850_bus[0].tx_buf[J1850_MSG_BUF_SIZE_TX]) end = (j1850_msg_buf_t *)j1850_bus[0].tx_buf;
                    //On overflow drop the last message
                    cli();
                    if(end != j1850_bus[0].tx_msg_start) {
                        j1850_bus[0].tx_msg_end = end;
                    }
                    sei();
                }
                
                j1850_bus[0].rx_msg_start ++;
                if(j1850_bus[0].rx_msg_start == &j1850_bus[0].rx_buf[J1850_MSG_BUF_SIZE_RX]) j1850_bus[0].rx_msg_start = (j1850_msg_buf_t *)j1850_bus[0].rx_buf;
            }
        }
    }
    return (0);
}
