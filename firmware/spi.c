/*
 * spi.c - Pi J1850 Interface SPI communications 
 */

#include <stdint.h>
#include <avr/interrupt.h>
#include "spi.h"
#include "main.h"
#include "j1850.h"

typedef struct ringbuf_t ringbuf_t;

struct ringbuf_t {
    volatile uint8_t buf[SPI_BUF_SIZE];
    volatile uint8_t *start;
    volatile uint8_t *end;
};

static ringbuf_t rx_buf;
static ringbuf_t tx_buf;

static uint8_t spi_status;
static uint8_t spi_cmd_status;
static uint8_t last_tmr_10ms;

ISR(SPI_STC_vect) {
    uint8_t byte = SPDR;
    
    switch(spi_status) {
        case 0x01:
            SPDR = *tx_buf.start;
            sei();
            
            //On underflow just keep sending the last byte
            uint8_t *start = (uint8_t *)tx_buf.start;
            if(start != tx_buf.end) {
                start ++;
                if(start == &tx_buf.buf[SPI_BUF_SIZE]) start = (uint8_t *)tx_buf.buf;
                tx_buf.start = start;
            }
            
            spi_status = 0x00;
            break;
        case 0x02:
            sei();
            uint8_t *end = (uint8_t *)rx_buf.end;
            *end = byte;
            
            end ++;
            if(end == &rx_buf.buf[SPI_BUF_SIZE]) end = (uint8_t *)rx_buf.buf;
            //On overflow just keep dropping the last byte
            if(end != rx_buf.start) rx_buf.end = end;
            
            spi_status = 0x00;
            break;
        default:
            if(byte == 0) SPDR = tx_buf.start != tx_buf.end;
            sei();
            spi_status = byte;
    }
}

inline int8_t spi_tx_push(uint8_t byte) {
    uint8_t *end = (uint8_t *)tx_buf.end;
    *end = byte;
    
    end ++;
    if(end == &tx_buf.buf[SPI_BUF_SIZE]) end = (uint8_t *)tx_buf.buf;
    
    //On overflow drop the last byte and let the caller know
    if(end != tx_buf.start) {
        cli();
        tx_buf.end = end;
        sei();
    }
    else return -1;
    
    return 0;
}

static inline void pop_j1850_to_spi(volatile j1850_bus_t *bus) {
    j1850_msg_buf_t *start = bus->rx_msg_start;
    cli(); 
    j1850_msg_buf_t *end = bus->rx_msg_end;
    sei();
    
    if(start != end) {
        //Let the requester know there's data
        spi_tx_push(0x01);
        
        uint8_t i;
        for(i=0; i<start->bytes; i++) {
            spi_tx_push(start->buf[i]);
        }
        
        bus->rx_msg_start ++;
        if(bus->rx_msg_start == &bus->rx_buf[J1850_MSG_BUF_SIZE_RX]) bus->rx_msg_start = (j1850_msg_buf_t *)bus->rx_buf;
    }
    //Let the requester know there's no data
    else spi_tx_push(0x00);
}

void spi_process(uint8_t tmr_10ms) {
    uint8_t *start = (uint8_t *)rx_buf.start;
    cli();
    uint8_t *end = (uint8_t *)rx_buf.end;
    sei();
    
    //If we haven't heard from anyone in 100ms, say so
    if((uint8_t)(tmr_10ms - last_tmr_10ms) > 10) {
        last_tmr_10ms = tmr_10ms;
        spi_active = 0;
    }
    
    if(start != end) {
        spi_active = 1;
        last_tmr_10ms = tmr_10ms;
        
        switch(spi_cmd_status) {
            j1850_msg_buf_t *msg;
            static uint8_t *byte;
            case 0x00:
                switch(*start) {
                    case 0x01:
                        spi_tx_push(sw_state);
                        break;
                    case 0x02:
                        spi_tx_push(pwr_state);
                        break;
                    case 0x03:
                        pop_j1850_to_spi(&j1850_bus[0]);
                        break;
                    case 0x04:
                        pop_j1850_to_spi(&j1850_bus[1]);
                        break;
                    case 0x05:
                        j1850_listen_bytes = 0;
                        break;
                    case 0x06:
                        spi_cmd_status = 0x01;
                        break;
                    case 0x07:
                        spi_cmd_status = 0x02;
                        break;
                    case 0x08:
                        spi_cmd_status = 0x03;
                        break;
                }
                break;
            case 0x01:
                j1850_listen_headers[j1850_listen_bytes] = *start;
                j1850_listen_bytes ++;
                spi_cmd_status = 0;
                break;
            case 0x02:
                j1850_bus[0].tx_msg_end->bytes = *start;
                byte = j1850_bus[0].tx_msg_end->buf;
                spi_cmd_status = 0x04;
                break;
            case 0x03:
                j1850_bus[1].tx_msg_end->bytes = *start;
                byte = j1850_bus[1].tx_msg_end->buf;
                spi_cmd_status = 0x05;
                break;
            case 0x04:
                msg = (j1850_msg_buf_t *)j1850_bus[0].tx_msg_end;
                
                *byte = *start;
                byte ++;
                
                if(byte == &msg->buf[msg->bytes]) {
                    *byte = j1850_crc(msg->buf, msg->bytes);
                    msg->bytes ++;
                    
                    msg ++;
                    if(msg == &j1850_bus[0].tx_buf[J1850_MSG_BUF_SIZE_TX]) msg = (j1850_msg_buf_t *)j1850_bus[0].tx_buf;
                    //On overflow drop the last message
                    cli();
                    if(msg != j1850_bus[0].tx_msg_start) {
                        j1850_bus[0].tx_msg_end = msg;
                    }
                    sei();
                    
                    spi_cmd_status = 0x00;
                }
                break;
            case 0x05:
                msg = (j1850_msg_buf_t *)j1850_bus[1].tx_msg_end;
                
                *byte = *start;
                byte ++;
                
                if(byte == &msg->buf[msg->bytes]) {
                    *byte = j1850_crc(msg->buf, msg->bytes);
                    msg->bytes ++;
                    
                    msg ++;
                    if(msg == &j1850_bus[1].tx_buf[J1850_MSG_BUF_SIZE_TX]) msg = (j1850_msg_buf_t *)j1850_bus[1].tx_buf;
                    //On overflow drop the last message
                    cli();
                    if(msg != j1850_bus[1].tx_msg_start) {
                        j1850_bus[1].tx_msg_end = msg;
                    }
                    sei();
        
                    spi_cmd_status = 0x00;
                }
                break;
        }
        
        start ++;
        if(start == &rx_buf.buf[SPI_BUF_SIZE]) start = (uint8_t *)rx_buf.buf;
        cli();
        rx_buf.start = start;
        sei();
    }
}

void spi_init_slave(void) {
    spi_status = 0;
    spi_cmd_status = 0;
    spi_active = 0;
    
    //Setup ring buffer pointers
    rx_buf.start = rx_buf.buf;
    rx_buf.end = rx_buf.start;
    tx_buf.start = tx_buf.buf;
    tx_buf.end = tx_buf.start;
    
    //MISO as OUTPUT
    MISO_DDR |= MISO_MSK;
    //Enable SPI and interrupt
    SPDR = 0;
    SPCR = (1<<SPE) | (1<<SPIE);
}
