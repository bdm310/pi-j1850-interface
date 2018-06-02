/*
 * main.h - Pi J1850 Interface
 */

#ifndef __MAIN_H__
#define __MAIN_H__

volatile uint8_t timer_10ms;
volatile uint8_t timer_1s;
volatile uint8_t cnt_10ms;

volatile uint8_t sw0_adc;
volatile uint8_t sw1_adc;
volatile uint8_t sw_new;
uint8_t sw_state;

#endif // __MAIN_H__
