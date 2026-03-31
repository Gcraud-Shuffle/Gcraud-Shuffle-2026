/*
 * File:   interrupt.hpp
 * Author: takum
 *
 * Created on March 13, 2026, 2:46 PM
 */

#ifndef PPS_INTERRUPT_HPP
#define PPS_INTERRUPT_HPP

#include "../src/u1.hpp"
#include "definitions.h"
#include "struct.hpp"
#include "algo.hpp"
#include "user.hpp"

void U1_Callback(uintptr_t);
void T2_Callback(uint32_t,uintptr_t);

extern uint8_t U1RxData;

#endif /* PPS_INTERRUPT_HPP */
