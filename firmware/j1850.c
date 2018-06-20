/*
 * j1850.c - J1850 interface
 */
#include "j1850.h"

static inline void set_ocr(j1850_bus_t *bus, uint8_t cnt) {
    if(bus == &j1850_bus[0]) {
        J1850_BUS0_OCR_REG = cnt;
        J1850_BUS0_OCF_REG |= J1850_BUS0_OCF_MSK;
        J1850_BUS0_OCIE_REG |= J1850_BUS0_OCIE_MSK;
    }
    else {
        J1850_BUS1_OCR_REG = cnt;
        J1850_BUS1_OCF_REG |= J1850_BUS1_OCF_MSK;
        J1850_BUS1_OCIE_REG |= J1850_BUS1_OCIE_MSK;
    }
}

static inline void stop_ocr(j1850_bus_t *bus) {
    if(bus == &j1850_bus[0]) J1850_BUS0_OCIE_REG &= ~J1850_BUS0_OCIE_MSK;
    else J1850_BUS1_OCIE_REG &= ~J1850_BUS1_OCIE_MSK;
}

static inline void set_port(j1850_bus_t *bus) {
    if(bus == &j1850_bus[0]) J1850_BUS0_PORT_REG |= J1850_BUS0_PORT_MSK;
    else J1850_BUS1_PORT_REG |= J1850_BUS1_PORT_MSK;
}

static inline void clear_port(j1850_bus_t *bus) {
    if(bus == &j1850_bus[0]) J1850_BUS0_PORT_REG &= ~J1850_BUS0_PORT_MSK;
    else J1850_BUS1_PORT_REG &= ~J1850_BUS1_PORT_MSK;
}

static inline void toggle_port(j1850_bus_t *bus) {
    if(bus == &j1850_bus[0]) J1850_BUS0_PORT_REG = J1850_BUS0_PORT_REG ^ J1850_BUS0_PORT_MSK;
    else J1850_BUS1_PORT_REG = J1850_BUS1_PORT_REG ^ J1850_BUS1_PORT_MSK;
}

static inline uint8_t get_port(j1850_bus_t *bus) {
    if(bus == &j1850_bus[0]) return J1850_BUS0_PORT_REG & J1850_BUS0_PORT_MSK;
    else return J1850_BUS1_PORT_REG & J1850_BUS1_PORT_MSK;
}

static inline uint8_t get_pin(j1850_bus_t *bus) {
    if(bus == &j1850_bus[0]) return J1850_BUS0_PIN_REG & J1850_BUS0_PIN_MSK;
    else return J1850_BUS1_PIN_REG & J1850_BUS1_PIN_MSK;
}

static inline void service_pcint(j1850_bus_t *bus, uint8_t pin, uint8_t tmr) {
    if(!(pin ^ bus->last_pin)) return;
    bus->last_pin = pin;
    
    uint8_t delta = tmr - bus->ltmr;
    bus->ltmr = tmr;
    
    switch(bus->state) {
        case 0:
            //Transition away from idle
            if(pin) bus->state = 0x01;
            break;
        case 1:
            //Check for SOF
            if(delta > RX_SOF_MAX || delta < RX_SOF_MIN) {
                bus->state = 0;
            }
            else {
                bus->state = 0x02;
                bus->rx_msg_end->bytes = 0;
                bus->bit_ptr = 0;
                bus->byte_ptr = bus->rx_msg_end->buf;
            }
            break;
        case 2:
            //Receive data bits
            if(bus->byte_ptr == bus->rx_msg_end->buf + J1850_MSG_SIZE) {
                //We've started the 13th byte or 
                //the pulse was too short/long, something went wrong
                stop_ocr(bus);
                bus->state = 0;
            }
            else if(delta > RX_SHORT_MIN && delta < RX_LONG_MAX) {
                //Setup EOD interrupt
                set_ocr(bus, tmr + RX_EOD_MIN);
                
                *bus->byte_ptr <<= 1;
                if((pin && delta > RX_LONG_MIN) || (!pin && delta < RX_SHORT_MAX)) {
                    //Passive or active 1
                    *bus->byte_ptr |= 1;
                }
                bus->bit_ptr ++;
                
                if(bus->bit_ptr == 8) {
                    bus->rx_msg_end->bytes ++;
                    bus->bit_ptr = 0;
                    bus->byte_ptr ++;
                }
            }
            break;
        case 11:
            //Pin changed while we were waiting for IFS, reset timer
            set_ocr(bus, tmr + TX_IFS);
            break;
        case 12:
            //Sending bits, check that we're not getting overridden
            if(!(pin) != !(get_port(bus))) {
                stop_ocr(bus);
                clear_port(bus);
                bus->state = 0;
            } 
            break;
    }
}

/*
 * Service the pin change ISR and direct it to the proper bus
 */
ISR(PCINT2_vect) {
    uint8_t tmr = TCNT2;
    sei();
    
    if(J1850_BUS0_PCINT_REG & J1850_BUS0_PCINT_MSK) {
        service_pcint( (j1850_bus_t *)&j1850_bus[0], 
                       get_pin((j1850_bus_t *)&j1850_bus[0]), 
                       tmr );
    }
    
    if(J1850_BUS1_PCINT_REG & J1850_BUS1_PCINT_MSK) {
        service_pcint( (j1850_bus_t *)&j1850_bus[1], 
                       get_pin((j1850_bus_t *)&j1850_bus[1]), 
                       tmr );
    }
}

static inline void service_ocr(j1850_bus_t *bus, uint8_t tmr) {
    switch(bus->state) {
        case 2:
            //Received EOD
            stop_ocr(bus);
            
            uint8_t match = 0;
            if(j1850_listen_bytes) {
                uint8_t i;
                for(i=0; i<j1850_listen_bytes; i++) {
                    if(bus->rx_msg_end->buf[0] == j1850_listen_headers[i]) {
                        match = 1;
                        break;
                    }
                }
            }
            else match = 1;
            
            if(match) {
                j1850_msg_buf_t *prev_end = bus->rx_msg_end;
                bus->rx_msg_end ++;
                if(bus->rx_msg_end == &bus->rx_buf[J1850_MSG_BUF_SIZE_RX]) bus->rx_msg_end = bus->rx_buf;
                if(bus->rx_msg_end == bus->rx_msg_start) bus->rx_msg_end = prev_end;
            }
            
            bus->state = 0;
            break;
        case 10:
            //Try to send something
            bus->state = 11;
            bus->bit_ptr = 0;
            bus->byte_ptr = bus->tx_msg_start->buf - 1;
            clear_port(bus);
            set_ocr(bus, tmr + TX_IFS);
            break;
        case 11:
            //Waiting for IFS
            
            if(get_pin(bus)) {
                //Bus isn't passive
                set_ocr(bus, tmr + TX_IFS);
            }
            else {
                //Successfully waited for IFS
                bus->state = 12;
                set_port(bus);
                set_ocr(bus, tmr + TX_SOF);
            }
            break;
        case 12:
            //Sending bits
            toggle_port(bus);
            
            if(bus->bit_ptr) bus->bit_ptr --;
            else if(bus->tx_msg_start->bytes) {
                bus->tx_msg_start->bytes --;
                bus->byte_ptr ++;
                bus->tx_byte = *bus->byte_ptr;
                bus->bit_ptr = 7;
            }
            else {
                //Done sending bits
                bus->state = 0;
                
                bus->tx_msg_start++;
                if(bus->tx_msg_start == &bus->tx_buf[J1850_MSG_BUF_SIZE_TX]) bus->tx_msg_start = bus->tx_buf;
                
                stop_ocr(bus);
                break;
            }
            
            if(!(bus->tx_byte & 0x80) != !get_port(bus)) set_ocr(bus, tmr + TX_LONG);
            else set_ocr(bus, tmr + TX_SHORT);
            
            bus->tx_byte <<= 1;
            break;
    }
}

/*
 * J1850 channel timer compare interrupts
 */
ISR(TIMER2_COMPA_vect) {
    uint8_t tmr = TCNT2;
    sei();
    
    service_ocr((j1850_bus_t *)&j1850_bus[0], tmr);
}

ISR(TIMER2_COMPB_vect) {
    uint8_t tmr = TCNT2;
    sei();
    
    service_ocr((j1850_bus_t *)&j1850_bus[1], tmr);
}

/*
 * Starts the interrupt system sending out the next message in the buffer
 */
void j1850_send_packet(uint8_t bus) {
    cli();
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
            if (bit_point & *byte_point)    // case for new bit = 1
            {
                if (crc_reg & 0x80)
                    poly=1; // define the polynomial
                else
                    poly=0x1c;
                crc_reg= ( (crc_reg << 1) | 1) ^ poly;
            }
            else        // case for new bit = 0
            {
                poly=0;
                if (crc_reg & 0x80)
                    poly=0x1d;
                crc_reg= (crc_reg << 1) ^ poly;
            }
        }
    }
    return ~crc_reg;    // Return CRC
}

/*
 * Housekeeping
 */
void j1850_process(void) {
    uint8_t bus;
    for(bus=0; bus<2; bus++) {
        cli();
        j1850_msg_buf_t *start = (j1850_msg_buf_t *)j1850_bus[bus].tx_msg_start;
        uint8_t state = j1850_bus[bus].state;
        sei();
        j1850_msg_buf_t *end = (j1850_msg_buf_t *)j1850_bus[bus].tx_msg_end;
    
        if((start != end) && state == 0) {
            j1850_send_packet(bus);
        }
    }
}

/*
 * Initialize all the J1850 stuff
 */
void j1850_init(void) {
    j1850_bus[0].rx_msg_start = (j1850_msg_buf_t *)j1850_bus[0].rx_buf;
    j1850_bus[0].rx_msg_end = j1850_bus[0].rx_msg_start;
    j1850_bus[0].tx_msg_start = (j1850_msg_buf_t *)j1850_bus[0].tx_buf;
    j1850_bus[0].tx_msg_end = j1850_bus[0].tx_msg_start;
    j1850_bus[1].rx_msg_start = (j1850_msg_buf_t *)j1850_bus[1].rx_buf;
    j1850_bus[1].rx_msg_end = j1850_bus[1].rx_msg_start;
    j1850_bus[1].tx_msg_start = (j1850_msg_buf_t *)j1850_bus[1].tx_buf;
    j1850_bus[1].tx_msg_end = j1850_bus[1].tx_msg_start;
    
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
