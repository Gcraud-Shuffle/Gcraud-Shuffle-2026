/*
 * UART.c
 *
 *  Created on: Dec 9, 2024
 *      Author: kinhi
 */

#include "UART.h"

volatile uint8_t CAMERA_flag = 0;
volatile uint8_t UART4_flag = 0;
volatile uint8_t ESP32_Comm_flag = 0;

ESP_data ESP32_TX_Data = {0};
ESP_data ESP32_RX_Data = {0};
bool ESP32_Failed_Connection = 0;

bool IR_Failed_Connection = 0;
bool LINE_Failed_Connection = 0;
bool MAIN_SUB_Failed_Connection = 0;

int16_t Ball_Theta = 0;
uint8_t Ball_Closeness = 0;
int8_t LINE_X = 0;
int8_t LINE_Y = 0;
uint8_t LINE_Side_Right;
uint8_t LINE_Side_Back;
uint8_t LINE_Side_Left;
bool LINE_Angel;
int16_t Goal_Blue_angle;
uint8_t Goal_Blue_angle_range;
uint8_t Goal_Blue_radius;
uint8_t Goal_Blue_size;
int16_t Goal_Yellow_angle;
uint8_t Goal_Yellow_angle_range;
uint8_t Goal_Yellow_radius;
uint8_t Goal_Yellow_size;
uint8_t Goal_RB;
int16_t Goal_Before[8] = {};
uint8_t Goal_Received_counter;
bool Goal_ava = 0;
uint8_t MainSub_Status = 0;
uint8_t MainSub_Ultrasonic_cm[3] = {};
uint16_t MainSub_Ultrasonic_mm[3] = {
    MAIN_SUB_INVALID_DISTANCE_MM,
    MAIN_SUB_INVALID_DISTANCE_MM,
    MAIN_SUB_INVALID_DISTANCE_MM};
bool MainSub_Ultrasonic_valid[3] = {};
uint8_t MainSub_Current4bit[4] = {};
uint16_t MainSub_Current_ADC12[4] = {};
uint8_t MainSub_Comm_Data[MAIN_SUB_COMM_PAYLOAD_MAX] = {};
uint8_t MainSub_Comm_Length = 0;
bool MainSub_Comm_Available = 0;

#define SENSOR_UART_REQUEST_BYTE 0x00U
#define SENSOR_UART_TIMEOUT_MS 2U
#define IR_RESPONSE_SIZE 2U
#define LINE_RESPONSE_SIZE 3U
#define MAIN_SUB_UART_TIMEOUT_MS 2U

static bool request_sensor_frame(UART_HandleTypeDef *uart,
                                 uint8_t *response,
                                 uint16_t response_size) {
  const uint8_t request = SENSOR_UART_REQUEST_BYTE;

  if ((uart == NULL) || (response == NULL) || (response_size == 0U)) {
    return false;
  }

  /*
   * The public sensor calls are synchronous. Interrupt reception followed by
   * a busy-wait made their timeout depend on SysTick running during UART error
   * interrupts, which is not guaranteed when a disconnected RX pin floats.
   */
  if (HAL_UART_AbortReceive(uart) != HAL_OK) {
    return false;
  }
  __HAL_UART_CLEAR_PEFLAG(uart);

  if (HAL_UART_Transmit(uart, &request, 1U, SENSOR_UART_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  if (HAL_UART_Receive(uart, response, response_size,
                       SENSOR_UART_TIMEOUT_MS) != HAL_OK) {
    (void)HAL_UART_AbortReceive(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);
    return false;
  }

  return true;
}

static bool main_sub_transaction(UART_HandleTypeDef *uart,
                                 uint8_t request,
                                 uint8_t *response,
                                 uint16_t response_size) {
  if ((uart == NULL) || (response == NULL) || (response_size == 0U)) {
    return false;
  }

  if (HAL_UART_AbortReceive(uart) != HAL_OK) {
    return false;
  }
  __HAL_UART_CLEAR_PEFLAG(uart);

  if (HAL_UART_Transmit(uart, &request, 1U, MAIN_SUB_UART_TIMEOUT_MS) != HAL_OK) {
    return false;
  }

  if (HAL_UART_Receive(uart, response, response_size,
                       MAIN_SUB_UART_TIMEOUT_MS) != HAL_OK) {
    (void)HAL_UART_AbortReceive(uart);
    __HAL_UART_CLEAR_PEFLAG(uart);
    return false;
  }

  return true;
}

void get_IR(UART_HandleTypeDef *uart) {
  /* Response: [angle encoded as 0..255, closeness as 0..255]. */
  uint8_t response[IR_RESPONSE_SIZE] = {0U};

  IR_Failed_Connection =
      !request_sensor_frame(uart, response, (uint16_t)sizeof(response));
  if (IR_Failed_Connection) {
    return;
  }

  Ball_Theta = (int16_t)response[0] * 360 / 256 - 178;
  Ball_Closeness = response[1];
}

void get_LINE(UART_HandleTypeDef *uart) {
  /*
   * Response: [X + 128, Y + 128, RR BB LL A0].
   * Each side uses two bits; A indicates that the circular line is visible.
   */
  uint8_t response[LINE_RESPONSE_SIZE] = {0U};

  LINE_Failed_Connection =
      !request_sensor_frame(uart, response, (uint16_t)sizeof(response));
  if (LINE_Failed_Connection) {
    return;
  }

  LINE_X = (int8_t)((int16_t)response[0] - 128);
  LINE_Y = (int8_t)((int16_t)response[1] - 128);
  LINE_Side_Right = response[2] & 0x03U;
  LINE_Side_Back = (response[2] >> 2) & 0x03U;
  LINE_Side_Left = (response[2] >> 4) & 0x03U;
  LINE_Angel = (response[2] & 0x40U) != 0U;
}

void get_MAIN_SUB(UART_HandleTypeDef *uart) {
  uint8_t response[MAIN_SUB_FIXED_RESPONSE_SIZE] = {0U};
  uint8_t status;
  uint8_t payload_length;
  uint8_t index;

  MAIN_SUB_Failed_Connection = false;
  MainSub_Comm_Available = false;
  MainSub_Comm_Length = 0U;

  if (!main_sub_transaction(uart, MAIN_SUB_REQUEST_GET_ALL, response,
                            (uint16_t)sizeof(response))) {
    MAIN_SUB_Failed_Connection = true;
    return;
  }

  if (response[0] != MAIN_SUB_RESPONSE_MAGIC) {
    MAIN_SUB_Failed_Connection = true;
    return;
  }

  status = response[1];

  for (index = 0U; index < 3U; index++) {
    uint8_t valid_bit = (uint8_t)(1U << index);

    MainSub_Ultrasonic_cm[index] = response[2U + index];
    if (((status & valid_bit) != 0U) &&
        (response[2U + index] != 0xFFU)) {
      MainSub_Ultrasonic_mm[index] = (uint16_t)response[2U + index] * 10U;
      MainSub_Ultrasonic_valid[index] = true;
    } else {
      MainSub_Ultrasonic_mm[index] = MAIN_SUB_INVALID_DISTANCE_MM;
      MainSub_Ultrasonic_valid[index] = false;
      status &= (uint8_t)~valid_bit;
    }
  }
  MainSub_Status = status;

  MainSub_Current4bit[0] = (response[5] >> 4) & 0x0FU;
  MainSub_Current4bit[1] = response[5] & 0x0FU;
  MainSub_Current4bit[2] = (response[6] >> 4) & 0x0FU;
  MainSub_Current4bit[3] = response[6] & 0x0FU;

  for (index = 0U; index < 4U; index++) {
    MainSub_Current_ADC12[index] = (uint16_t)MainSub_Current4bit[index] << 8;
  }

  payload_length = response[7];
  if (payload_length > MAIN_SUB_COMM_PAYLOAD_MAX) {
    MAIN_SUB_Failed_Connection = true;
    return;
  }

  if (((MainSub_Status & MAIN_SUB_STATUS_COMM_DATA) != 0U) &&
      (payload_length > 0U)) {
    if (!main_sub_transaction(uart, MAIN_SUB_REQUEST_GET_COMM,
                              MainSub_Comm_Data, payload_length)) {
      MAIN_SUB_Failed_Connection = true;
      return;
    }

    MainSub_Comm_Length = payload_length;
    MainSub_Comm_Available = true;
  }
}

void get_ESP32(UART_HandleTypeDef *ESP32_uart_ch, ESP_data tx_data) {
  uint8_t RB = 0;
  uint32_t start_cnt;
  uint32_t now_cnt;
  uint32_t elapsed;
  UART4_flag = 0;
  ESP32_Failed_Connection = 0;

  if (ESP32_uart_ch->RxState != HAL_UART_STATE_READY) {
    HAL_UART_AbortReceive_IT(ESP32_uart_ch);
  }

  __HAL_UART_CLEAR_OREFLAG(ESP32_uart_ch);
  HAL_UART_Receive_IT(ESP32_uart_ch, &RB, 1);

  HAL_UART_Transmit(ESP32_uart_ch, &tx_data.byte, 1, 100);

  start_cnt = HAL_GetTick();

  while (!UART4_flag) {
    now_cnt = HAL_GetTick();
    elapsed = now_cnt - start_cnt;

    if (elapsed > TIMEOUT_CNT) {
      HAL_UART_AbortReceive_IT(ESP32_uart_ch);
      UART4_flag = 1;
      ESP32_Failed_Connection = 1;
      break;
    }

    if (__HAL_UART_GET_FLAG(ESP32_uart_ch, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(ESP32_uart_ch);
      HAL_UART_AbortReceive_IT(ESP32_uart_ch);
      HAL_UART_Receive_IT(ESP32_uart_ch, &RB, 1);
    }
  }

  if (UART4_flag && !ESP32_Failed_Connection) {
    ESP32_RX_Data.byte = RB;
  }
}
