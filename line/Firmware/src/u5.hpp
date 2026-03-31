/* 
 * File:   u5.hpp
 * Author: takum
 *
 * Created on May 22, 2025, 11:39 PM
 */

#ifndef U5_HPP
#define	U5_HPP

#include <vector>
#include <functional>
#include "definitions.h"

class UART5_class{
    public:
        void init();
        UART5_class& Write(uint8_t data);
        UART5_class& head(){Write(0x80);return *this;}
        UART5_class& tail(){Write(0x7f);return *this;}
        UART5_class& send_uint32(uint32_t data){
            Write((data>> 0)&0xff);
            Write((data>> 8)&0xff);
            Write((data>>16)&0xff);
            Write((data>>24)&0xff);
            return *this;
        }
        UART5_class& send_int32(int32_t data){
            Write((data>> 0)&0xff);
            Write((data>> 8)&0xff);
            Write((data>>16)&0xff);
            Write((data>>24)&0xff);
            return *this;
        }
        uint8_t tx_data;
        uint8_t rx_data;
};

extern UART5_class U5;

#endif	/* U5_HPP */
