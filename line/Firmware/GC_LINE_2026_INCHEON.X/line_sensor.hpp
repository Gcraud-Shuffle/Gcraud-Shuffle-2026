/*
 * File:   line_sensor.hpp
 * Author: takum
 *
 * Created on March 9, 2026, 2:32 PM
 */

#ifndef LINE_SENSOR_HPP
#define LINE_SENSOR_HPP

#include <stdint.h>

#include "definitions.h"

extern uint16_t ANALOG_VALUE;

extern volatile uint32_t *S_port[];
extern uint32_t S_mask[];
#define get_Angel(i) (!(bool)(*S_port[(i)] & S_mask[(i)]))

#define get_out_val(I, M, E) ((I) ? 3 : ((M) ? 2 : ((E) ? 1 : 0)))

#define AngelA (~PORTA & 0xC102)  // 4:RA1,RA8,RA14,RA15
#define AngelB (~PORTB & 0xFF8C)  // 11:RB2,RB3,RB7,RB8,RB9,RB10,RB11,RB12,RB13,RB14,RB15
#define AngelC (~PORTC & 0x21C6)  // 6:RC1,RC2,RC6,RC7,RC8,RC13
#define AngelD (~PORTD & 0xF106)  // 7:RD1,RD2,RD8,RD12,RD13,RD14,RD15
#define AngelE 0                  // 0:
#define AngelF (~PORTF & 0x0220)  // 2:RF5,RF9
#define AngelG (~PORTG & 0x0180)  // 2:RG7,RG8

#define Angel_or ((bool)(AngelA | AngelB | AngelC | AngelD | AngelE | AngelF | AngelG))

#define RI (!PORTBbits.RB4)                                                                                             // inside : RB4
#define RM (!PORTFbits.RF12)                                                                                            // middle : RF12
#define RE ((bool)((~PORTA & 0x0010) | (~PORTC & 0x0801) | (~PORTE & 0x0003) | (~PORTF & 0x0400) | (~PORTG & 0x0800)))  // edge : RA4, RC0, RC11, RE0, RE1, RF10, RG11
#define R get_out_val(RI, RM, RE)

#define LI (!PORTGbits.RG14)                                                                                            // inside : RG14
#define LM (!PORTFbits.RF7)                                                                                             // middle : RF7
#define LE ((bool)((~PORTA & 0x0080) | (~PORTC & 0x0200) | (~PORTG & 0xB003)))                                          // edge : RA7, RC9, RG0, RG1, RG12, RG13, RG15
#define L get_out_val(LI, LM, LE)

#define BI (!PORTDbits.RD4)                                                                                             // inside : RD4
#define BM (!PORTDbits.RD3)                                                                                             // middle : RD3
#define BE ((bool)((~PORTA & 0x1801) | (~PORTE & 0x0300)))                                                              // edge : RA0, RA11, RA12, RE8, RE9
#define B (get_out_val(BI, BM, BE))

#endif /* LINE_SENSOR_HPP */
