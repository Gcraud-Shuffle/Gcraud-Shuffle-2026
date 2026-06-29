/*
 * queue.cpp
 *
 *  Created on: Oct 3, 2024
 *      Author: kinhi
 */

#include "queue.hpp"

Queue::Queue() {}

void Queue::push(uint8_t position, bool input) {
    if (PushIndex[position] == NS) {
        PushIndex[position] = 0;
    }

    contents[position][PushIndex[position]] = input;
    sum[position] += input;

    PushIndex[position]++;
    TrueIndex[position]++;
}

bool Queue::pop(uint8_t position) {
    if (PopIndex[position] == NS) {
        PopIndex[position] = 0;
    }

    bool output = contents[position][PopIndex[position]];
    sum[position] -= output;

    PopIndex[position]++;
    TrueIndex[position]--;

    return output;
}

bool Queue::rear(uint8_t position) { return contents[position][PushIndex[position] - 1]; }

bool Queue::head(uint8_t position) { return contents[position][PopIndex[position] - 1]; }

bool Queue::pop_push(uint8_t position, bool input) {
    bool output = pop(position);  // クラス内メソッドなので、this-> は省略可能
    push(position, input);

    return output;
}

short Queue::get_sum(uint8_t position) { return sum[position]; }
