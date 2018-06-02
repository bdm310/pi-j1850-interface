/*
 * ringbuffer.h - Pi J1850 Interface SPI communications 
 */

#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#include <avr/interrupt.h>
#include <stdint.h>

#define RING_BUF_SIZE 64

typedef struct ring_buf_t ring_buf_t;

struct ring_buf_t {
	uint8_t buffer[RING_BUF_SIZE+1];
	uint8_t start;
	uint8_t end;
};

inline uint8_t ring_buf_empty(ring_buf_t *buf);
void ring_buf_push(ring_buf_t *buf, uint8_t data);
uint8_t ring_buf_pop(ring_buf_t *buf);
uint8_t ring_buf_bytes(ring_buf_t *buf);

#endif // __RINGBUFFER_H__
