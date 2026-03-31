/*
 * UART.h
 *
 *  Created on: Dec 9, 2024
 *      Author: kinhi
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "main.h"
#include <stdbool.h>
#include <stdio.h>

enum { IR_THETAreq, IR_CLOSENESSreq, LINE_Xreq, LINE_Yreq, LINE_SIDEreq };
#define TIMEOUT_CNT 1
// extern int flagReceived[6];
extern volatile uint8_t IR_flag;
extern volatile uint8_t LINE_flag;
extern volatile uint8_t CAMERA_flag;
extern volatile uint8_t UART4_flag;
extern volatile uint8_t ESP32_Comm_flag;

// --- Role Management ENUMs ---
enum RoleState {
    ROLE_KEEPER = 0,
    ROLE_FORWARD = 1
};

enum RobotState {
    STATE_STANDALONE = 0,
    STATE_COORDINATED = 1,
    STATE_RECOVERING = 2
};

typedef union {
  uint8_t byte;
  struct {
    unsigned char local_ACK : 1; // 0: NACK /1: ACK
    unsigned char role : 1; // FORWARD or KEEPER
    unsigned char youWereDead : 1; // お前はすでに死んでいる
    unsigned char ImDIE : 1; // Oh,I'm die.Thank you forever
    unsigned char goal_pos : 2; // 0:ゴール前にいない /1:ゴール右 /2:ゴール前 /3:ゴール左
    unsigned char hold_flag : 1;
    unsigned char no_connection : 1; // connection timeout
  };
} ESP_data;

extern ESP_data ESP32_TX_Data;
extern ESP_data ESP32_RX_Data;
extern bool ESP32_Failed_Connection;

extern uint8_t IRorLINE; // IR:0, LINE:1
extern UART_HandleTypeDef *Now_ch;
extern uint8_t TB;
extern bool IR_Failed_Connection;
extern bool LINE_Failed_Connection;
extern bool Fisrt_Line;

extern int16_t Ball_Theta;
extern uint8_t Ball_Closeness;
// extern int16_t LINE_Angle;
// extern uint8_t LINE_Depth;
extern int8_t LINE_X;
extern int8_t LINE_Y;
extern uint8_t LINE_Side_Right;
extern uint8_t LINE_Side_Back;
extern uint8_t LINE_Side_Left;
extern bool LINE_Angel;
// extern int

extern int16_t Goal_Blue_angle;
extern uint8_t Goal_Blue_angle_range;
extern uint8_t Goal_Blue_radius;
extern uint8_t Goal_Blue_size;
extern int16_t Goal_Yellow_angle;
extern uint8_t Goal_Yellow_angle_range;
extern uint8_t Goal_Yellow_radius;
extern uint8_t Goal_Yellow_size;
extern uint8_t Goal_RB;
extern int16_t Goal_Before[8];
extern uint8_t Goal_Received_counter;
extern bool Goal_ava;

void get_IR(UART_HandleTypeDef *IR_uart_ch, TIM_HandleTypeDef *htim_intr);
void get_LINE(UART_HandleTypeDef *LINE_uart_ch, TIM_HandleTypeDef *htim_intr);
void get_ESP32(UART_HandleTypeDef *ESP32_uart_ch, ESP_data tx_data);

#endif /* INC_UART_H_ */
