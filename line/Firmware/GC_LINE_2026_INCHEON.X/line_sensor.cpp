/*
 * File:   line_sensor.cpp
 * Author: takum
 *
 * Created on March 9, 2026, 2:32 PM
 */

#include "line_sensor.hpp"

uint16_t ANALOG_VALUE;

volatile uint32_t *S_port[32] = {&PORTD, &PORTA, &PORTD, &PORTF, &PORTA, &PORTA, &PORTD, &PORTB, &PORTC, &PORTB, &PORTB, &PORTC, &PORTD, &PORTC, &PORTD, &PORTC,
                                 &PORTB, &PORTB, &PORTB, &PORTB, &PORTB, &PORTB, &PORTD, &PORTD, &PORTG, &PORTG, &PORTB, &PORTA, &PORTB, &PORTF, &PORTC, &PORTC};

uint32_t S_mask[32] = {1 << 15, 1 << 8,  1 << 14, 1 << 5,  1 << 14, 1 << 15, 1 << 8,  1 << 7,  1 << 13, 1 << 8,  1 << 9,  1 << 6,  1 << 12, 1 << 7,  1 << 13, 1 << 8,
                       1 << 10, 1 << 11, 1 << 12, 1 << 13, 1 << 14, 1 << 15, 1 << 1,  1 << 2,  1 << 7,  1 << 8,  1 << 2,  1 << 1,  1 << 3,  1 << 9,  1 << 1,  1 << 2};
