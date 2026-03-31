/*
 * File:   u1.cpp
 * Author: takum
 *
 * Created on July 19, 2025, 04:45 AM
 */
#include "u1.hpp"

UART1_class U1;

void UART1_tx_callback(uintptr_t) {}

void UART1_class::init() { UART1_WriteCallbackRegister(UART1_tx_callback, 1); }

UART1_class &UART1_class::Write(uint8_t data) {
    tx_data = data;
    while (!UART1_Write(&tx_data, 1));
    return *this;
}
