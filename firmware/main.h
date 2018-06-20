/*
 * main.h - Pi J1850 Interface
 */

#ifndef __MAIN_H__
#define __MAIN_H__

volatile uint8_t tmr_10ms;
volatile uint8_t tmr_1s;

uint8_t sw_state;
uint8_t pwr_state;

#define set0 PORTC |= (1<<PORTC2);
#define clr0 PORTC &= ~(1<<PORTC2);
#define set1 PORTC |= (1<<PORTC3);
#define clr1 PORTC &= ~(1<<PORTC3);

#define F_CPU 8000000L

#define ACC_REG PINB
#define ACC_MSK (1<<PINB0)
#define PI_REG PINC
#define PI_MSK (1<<PINC5)

#define POWER_STATUS_BYTE ((PINB & (1<<PINB0)) != 0) + (((PINC & (1<<PINC5)) != 0)<<1)
#define POWER_HOLD_PORT PORTD
#define POWER_HOLD_DDR DDRD
#define POWER_HOLD_MSK (1<<PORTD4)

#define SW_THRESH_0 27
#define SW_THRESH_1 75
#define SW_THRESH_2 119
#define SW_THRESH_3 169
#define SW_THRESH_4 213

#endif // __MAIN_H__
