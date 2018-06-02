/*
 * ringbuffer.c - Generic ring buffer
 */

#include "ringbuffer.h"

inline uint8_t ring_buf_empty(ring_buf_t *buf) {
	uint8_t empty;
	
	cli();
	
	empty = buf->start == buf->end;
	
	sei();
	
	return empty;
}

void ring_buf_push(ring_buf_t *buf, uint8_t data) {
	cli();
	
	buf->buffer[buf->end] = data;
	
	buf->end ++;
	if(buf->end > RING_BUF_SIZE) buf->end = 0;
	if(buf->end == buf->start) {
		buf->start ++;
		if(buf->start > RING_BUF_SIZE) buf->start = 0;
	}
	
	sei();
}

uint8_t ring_buf_pop(ring_buf_t *buf){
	uint8_t byte = 0x00;
	
	cli();
	
	if(!ring_buf_empty(buf)) {
		byte = buf->buffer[buf->start];
		
		buf->start ++;
		if(buf->start > RING_BUF_SIZE) buf->start = 0;
	}
	
	sei();

	return byte;
}

uint8_t ring_buf_bytes(ring_buf_t *buf){
	uint8_t bytes;
	
	cli();
	
	if(buf->start <= buf->end)	bytes = buf->end - buf->start;
	else bytes = buf->end + (RING_BUF_SIZE+1 - buf->start);
	
	sei();
	
	return bytes;
}
