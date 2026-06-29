/*
 * app.hpp
 *
 *  Created on: Jun 1, 2026
 *      Author: taku-256
 */

#ifndef INC_APP_HPP_
#define INC_APP_HPP_
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "queue.hpp"
#include "stm32g4xx_hal.h"

int app(UART_HandleTypeDef *hlpuart1, UART_HandleTypeDef *huart1, /*uint8_t *Send_Data,*/ uint8_t *Sendtheta, uint8_t *Send_R);

// ball location
typedef struct {
    float x, y;
    uint8_t r;
    uint8_t theta;
} Ball;

#endif /* INC_APP_HPP_ */
