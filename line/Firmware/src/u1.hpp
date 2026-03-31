/*
 * File:   u1.hpp
 * Author: takum
 *
 * Created on May 22, 2025, 11:39 PM
 */

#ifndef U1_HPP
#define U1_HPP

#include <functional>
#include <vector>

#include "definitions.h"

class UART1_class {
public:
    void init();
    UART1_class &Write(uint8_t data);
    UART1_class &head() {
        Write(0x80);
        return *this;
    }
    UART1_class &tail() {
        Write(0x7f);
        return *this;
    }
    UART1_class &send_uint32(uint32_t data) {
        Write((data >> 0) & 0xff);
        Write((data >> 8) & 0xff);
        Write((data >> 16) & 0xff);
        Write((data >> 24) & 0xff);
        return *this;
    }
    UART1_class &send_int32(int32_t data) {
        Write((data >> 0) & 0xff);
        Write((data >> 8) & 0xff);
        Write((data >> 16) & 0xff);
        Write((data >> 24) & 0xff);
        return *this;
    }
    uint8_t tx_data;
    uint8_t rx_data;
};

extern UART1_class U1;

#endif /* U1_HPP */
