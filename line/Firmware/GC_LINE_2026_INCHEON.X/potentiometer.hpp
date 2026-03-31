/*
 * File:   potentiometer.hpp
 * Author: takum
 *
 * Created on March 5, 2026, 3:35 PM
 */

#ifndef POTENTIOMETER_HPP
#define POTENTIOMETER_HPP

#include "definitions.h"

/*****************************************************************************
Note:I2C Address
MCP4017 - 0b0101111
MCP4018 - 0b0101111
MCP4019 - 0b0101111
*****************************************************************************/

#define MCP4018_Address 0b0101111

void set2potentionmeter(uint8_t val);

#endif /* POTENTIOMETER_HPP */
