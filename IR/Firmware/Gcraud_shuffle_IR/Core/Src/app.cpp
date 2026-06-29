/*
 * app.cpp
 *
 *  Created on: Jun 1, 2026
 *      Author: taku-256
 */
#include "app.hpp"

#define R_THRESH 40
#define TAU 256.0

#define IGP(num) IR##num##_GPIO_Port
#define IP(num) IR##num##_Pin

GPIO_TypeDef *IGPA[24] = {IGP(1),  IGP(2),  IGP(3),  IGP(4),  IGP(5),  IGP(6),  IGP(7),  IGP(8),  IGP(9),  IGP(10), IGP(11), IGP(12),
                          IGP(13), IGP(14), IGP(15), IGP(16), IGP(17), IGP(18), IGP(19), IGP(20), IGP(21), IGP(22), IGP(23), IGP(24)};
uint16_t IPA[24] = {IP(1), IP(2), IP(3), IP(4), IP(5), IP(6), IP(7), IP(8), IP(9), IP(10), IP(11), IP(12),
					IP(13), IP(14), IP(15), IP(16), IP(17), IP(18), IP(19), IP(20), IP(21), IP(22), IP(23), IP(24)};

#define read_IR(IRchannel) (HAL_GPIO_ReadPin(IGPA[IRchannel], IPA[IRchannel]))

Queue queueIR;
Ball ball;

void get_all_IR() {
    for (int i = 0; i < NP; i++) {
        queueIR.pop_push(i, (int)(!(bool)read_IR(i)));
    }
}

double Ball_R = 0.0,now_Ball_R = 0.0;
//uint16_t allsum = 0;

int app(UART_HandleTypeDef *hlpuart1, UART_HandleTypeDef *huart1, /*uint8_t *Send_Data,*/ uint8_t *Sendtheta, uint8_t *Send_R) {
    // define const value of sin and cos.
    float SIN[NP];
    float COS[NP];

    for (int i = 0; i < NP; i++) {
        COS[i] = cos(i * M_PI * 2 / NP);
        SIN[i] = sin(i * M_PI * 2 / NP);
    }

    float BIN_8 = 255.0;

loop:
    get_all_IR();

    ball.x = 0.0;
    ball.y = 0.0;
//    allsum = 0;			//for debug

    for (int i = 0; i < NP; i++) {
        ball.x += (float)queueIR.get_sum(i) * COS[i];
        ball.y += (float)queueIR.get_sum(i) * SIN[i];
//        allsum += queueIR.get_sum(i);		//for debug
    }

    ball.theta = (uint8_t)(atan2(ball.y, ball.x) * BIN_8 / (2 * M_PI) + BIN_8 / 2);
    now_Ball_R = sqrt(ball.y*ball.y+ball.x*ball.x);
    Ball_R += (now_Ball_R - Ball_R)/TAU;
    ball.r = (uint8_t)(Ball_R * BIN_8 / (NS * MAX_sum));

    *Sendtheta = ball.theta;
    *Send_R = ball.r;

    goto loop;
    return -1;
}
