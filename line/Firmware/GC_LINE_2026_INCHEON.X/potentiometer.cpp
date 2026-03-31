/*
 * File:   potentiometer.cpp
 * Author: takum
 *
 * Created on March 5, 2026, 3:35 PM
 */

#include "potentiometer.hpp"

void set2potentionmeter(uint8_t val) {
    while (I2C3_IsBusy()) {}
    I2C3_Write(MCP4018_Address, &val, 1);
}
