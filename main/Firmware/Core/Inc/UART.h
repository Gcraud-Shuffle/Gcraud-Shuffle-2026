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

#define MAIN_SUB_REQUEST_GET_ALL 0xA5u
#define MAIN_SUB_REQUEST_GET_COMM 0xA6u
#define MAIN_SUB_RESPONSE_MAGIC 0x5Au
#define MAIN_SUB_FIXED_RESPONSE_SIZE 8u
#define MAIN_SUB_COMM_PAYLOAD_MAX 32u
#define MAIN_SUB_INVALID_DISTANCE_MM 0xFFFFu
#define MAIN_SUB_STATUS_US1_VALID (1u << 0)
#define MAIN_SUB_STATUS_US2_VALID (1u << 1)
#define MAIN_SUB_STATUS_US3_VALID (1u << 2)
#define MAIN_SUB_STATUS_CURRENT_VALID (1u << 3)
#define MAIN_SUB_STATUS_COMM_OVERFLOW (1u << 4)
#define MAIN_SUB_STATUS_COMM_UART_ERR (1u << 5)
#define MAIN_SUB_STATUS_COMM_DATA (1u << 6)
#define MAIN_SUB_US_SAFE_STATE_NO_DATA 0u
#define MAIN_SUB_US_SAFE_STATE_FRESH 1u
#define MAIN_SUB_US_SAFE_STATE_HOLD 2u
#define MAIN_SUB_US_SAFE_STATE_COMM_ERROR 3u
#define TIMEOUT_CNT 1

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
    unsigned char local_ACK : 1; // 0: NACK / 1: ACK
    unsigned char role : 1; // FORWARD or KEEPER
    unsigned char youWereDead : 1;
    unsigned char ImDIE : 1;
    unsigned char goal_pos : 2;
    unsigned char hold_flag : 1;
    unsigned char no_connection : 1;
  };
} ESP_data;

extern ESP_data ESP32_TX_Data;
extern ESP_data ESP32_RX_Data;
extern bool ESP32_Failed_Connection;

extern bool IR_Failed_Connection;
extern bool LINE_Failed_Connection;
extern bool MAIN_SUB_Failed_Connection;

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
extern uint8_t MainSub_Status;
extern uint8_t MainSub_Ultrasonic_cm[3];
extern uint16_t MainSub_Ultrasonic_mm[3];
extern bool MainSub_Ultrasonic_valid[3];
extern uint16_t MainSub_SafeUltrasonic_mm[3];
extern bool MainSub_SafeUltrasonic_valid[3];
extern uint8_t MainSub_SafeUltrasonic_state[3];
extern uint16_t MainSub_SafeUltrasonic_hold_count[3];
extern uint8_t MainSub_Current4bit[4];
extern uint16_t MainSub_Current_ADC12[4];
extern uint8_t MainSub_Comm_Data[MAIN_SUB_COMM_PAYLOAD_MAX];
extern uint8_t MainSub_Comm_Length;
extern bool MainSub_Comm_Available;

void get_IR(UART_HandleTypeDef *uart);
void get_LINE(UART_HandleTypeDef *uart);
void get_ESP32(UART_HandleTypeDef *ESP32_uart_ch, ESP_data tx_data);
void get_MAIN_SUB(UART_HandleTypeDef *uart);

#endif /* INC_UART_H_ */
