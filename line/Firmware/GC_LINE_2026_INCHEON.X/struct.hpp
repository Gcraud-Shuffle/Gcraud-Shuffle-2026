/*
 * File:   struct.hpp
 * Author: takum
 *
 * Created on March 6, 2026, 12:52 PM
 */

#ifndef STRUCT_HPP
#define STRUCT_HPP

#include <stdint.h>

typedef union {
    uint8_t data[3];
    struct {
        int8_t linex;
        int8_t liney;
        union {
            uint8_t D3;
            struct {
                unsigned char right : 2;
                unsigned char back : 2;
                unsigned char left : 2;
                unsigned char Angel_flag : 1;
                unsigned char ImDIE_flag : 1;
            };
        };
    };
} u_line_data;

extern u_line_data UART_Data;

#endif /* STRUCT_HPP */
