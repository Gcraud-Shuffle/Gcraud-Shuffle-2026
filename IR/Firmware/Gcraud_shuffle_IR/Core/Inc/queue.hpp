/*
 * queue.hpp
 *
 *  Created on: Oct 3, 2024
 *      Author: kinhi
 */

#ifndef INC_QUEUE_HPP_
#define INC_QUEUE_HPP_

#include "stm32g4xx_hal.h"

#define NP 24
#define NS 50
#define MAX_sum 4.5
class Queue {
private:
    bool contents[NP][NS] = {};
    short PopIndex[NP];
    short PushIndex[NP];
    short TrueIndex[NP];
    short sum[NP] = {};

public:
    Queue(void);
    void push(uint8_t position, bool input);
    bool pop(uint8_t position);
    bool rear(uint8_t position);
    bool head(uint8_t position);
    bool pop_push(uint8_t position, bool input);
    bool check_content(uint8_t position, short index);
    short get_sum(uint8_t position);
};

#endif /* INC_QUEUE_HPP_ */
