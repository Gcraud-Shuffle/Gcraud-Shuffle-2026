/* 
 * File:   u5.cpp
 * Author: takum
 *
 * Created on July 19, 2025, 04:45 AM
 */
#include "u5.hpp"

UART5_class U5;

void UART5_tx_callback(uintptr_t){}

void UART5_class::init(){
    UART5_WriteCallbackRegister(UART5_tx_callback,1);
}

UART5_class& UART5_class::Write(uint8_t data){
    tx_data = data;
    while(!UART5_Write(&tx_data,1));
    return *this;
}
