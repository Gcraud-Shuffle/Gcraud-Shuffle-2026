/*
 * UART.c
 *
 *  Created on: Dec 9, 2024
 *      Author: kinhi
 */

#include "UART.h"

volatile uint8_t IR_flag = 0;
volatile uint8_t LINE_flag = 0;
volatile uint8_t CAMERA_flag = 0;
volatile uint8_t UART4_flag = 0;
volatile uint8_t ESP32_Comm_flag = 0;

ESP_data ESP32_TX_Data = {0};
ESP_data ESP32_RX_Data = {0};
bool ESP32_Failed_Connection = 0;

uint8_t IRorLINE;
UART_HandleTypeDef *Now_ch = NULL;
bool IR_Failed_Connection = 0;
bool LINE_Failed_Connection = 0;
uint8_t TB = 0;
bool Fisrt_Line = 1;

int16_t Ball_Theta = 0;
uint8_t Ball_Closeness = 0;
// int16_t LINE_Angle = 0;
// uint8_t LINE_Depth = 0;
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

void get_IR(UART_HandleTypeDef *IR_uart_ch, TIM_HandleTypeDef *htim_intr) {

  // for debug
  //	Ball_Theta = 1;
  //	Ball_Closeness = 2;

  uint32_t start_cnt;
  uint32_t now_cnt;
  uint32_t elapsed;
  uint8_t RB[2] = {0};
  uint8_t TB = 0;
  IR_flag = 0; // init received flag
  IR_Failed_Connection = 0;

  if (IR_uart_ch->RxState != HAL_UART_STATE_READY) {
    HAL_UART_AbortReceive_IT(IR_uart_ch);
  }

  __HAL_UART_CLEAR_OREFLAG(IR_uart_ch);
  HAL_UART_Receive_IT(IR_uart_ch, RB, 2); // Reveived mode

  HAL_UART_Transmit(IR_uart_ch, &TB, 1, 100); // send request

  start_cnt = HAL_GetTick(); // 確実に動いているSystickタイマーを使用

  while (!IR_flag) { // wait for return
    now_cnt = HAL_GetTick();
    elapsed = now_cnt - start_cnt;

    if (elapsed > TIMEOUT_CNT) {
      HAL_UART_AbortReceive_IT(IR_uart_ch);
      IR_Failed_Connection = 1;
      break;
    }

    if (__HAL_UART_GET_FLAG(IR_uart_ch, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(IR_uart_ch);
      HAL_UART_AbortReceive_IT(IR_uart_ch);
      HAL_UART_Receive_IT(IR_uart_ch, RB, 2);
    }
  }

  if (IR_flag && !IR_Failed_Connection) {
    Ball_Theta = (int16_t)RB[0] * 360 / 256 - 178;
    Ball_Closeness = RB[1] & 0x7F; // パリティービット(MSB)を除去して0~127の値にする
  }
}



void get_LINE(UART_HandleTypeDef *LINE_uart_ch, TIM_HandleTypeDef *htim_intr) {

  uint8_t RB[3] = {0};
  uint32_t start_cnt;
  uint32_t now_cnt;
  uint32_t elapsed;
  TB = 0;
  LINE_flag = 0;
  LINE_Failed_Connection = 0;

  // もし前の受信が何らかの理由で終わっていなければ強制終了してリセットする
  if (LINE_uart_ch->RxState != HAL_UART_STATE_READY) {
    HAL_UART_AbortReceive_IT(LINE_uart_ch);
  }

  __HAL_UART_CLEAR_OREFLAG(LINE_uart_ch); // 念のため開始前にOREクリア
  HAL_UART_Receive_IT(LINE_uart_ch, RB, 3); // 安定動作のためIT(割り込み)に戻す

  HAL_UART_Transmit(LINE_uart_ch, &TB, 1, 100);

  start_cnt = HAL_GetTick(); // 確実に動いているSystickタイマーを使用

  while (!LINE_flag) {
    now_cnt = HAL_GetTick();
    elapsed = now_cnt - start_cnt;

    if (elapsed > TIMEOUT_CNT) {
      HAL_UART_AbortReceive_IT(LINE_uart_ch); // IT受信を強制停止
      LINE_flag = 1; // タイムアウトでもwhileを抜けるように強制フラグ立て
      LINE_Failed_Connection = 1; // データは無効化する
      break;
    }

    // もし高速通信によるオーバーランエラー(ORE)が発生してHALドライバが沈黙した場合に備え、手動でフラグをクリアする
    if (__HAL_UART_GET_FLAG(LINE_uart_ch, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(LINE_uart_ch);
      HAL_UART_AbortReceive_IT(LINE_uart_ch); // 一度アボートして
      HAL_UART_Receive_IT(LINE_uart_ch, RB, 3); // 即座に受信再開
    }
  }

	if (LINE_flag && !LINE_Failed_Connection) {

    LINE_X = (int8_t)RB[0] - 128;

    LINE_Y = (int8_t)RB[1] - 128;

    LINE_Side_Right = (RB[2] & 0b00000011);
    LINE_Side_Back = (RB[2] & 0b00001100) >> 2;
    LINE_Side_Left = (RB[2] & 0b00110000) >> 4;
    LINE_Angel = ((RB[2] & 0b01000000) > 0);
  }
}

void get_ESP32(UART_HandleTypeDef *ESP32_uart_ch, ESP_data tx_data) {

  uint8_t RB = 0;
  uint32_t start_cnt;
  uint32_t now_cnt;
  uint32_t elapsed;
  UART4_flag = 0;
  ESP32_Failed_Connection = 0;

  // もし前の受信が何らかの理由で終わっていなければ強制終了してリセットする
  if (ESP32_uart_ch->RxState != HAL_UART_STATE_READY) {
    HAL_UART_AbortReceive_IT(ESP32_uart_ch);
  }

  __HAL_UART_CLEAR_OREFLAG(ESP32_uart_ch); // 念のため開始前にOREクリア
  HAL_UART_Receive_IT(ESP32_uart_ch, &RB, 1); // 1byte受信待機

  HAL_UART_Transmit(ESP32_uart_ch, &tx_data.byte, 1, 100);

  start_cnt = HAL_GetTick(); // 確実に動いているSystickタイマーを使用

  while (!UART4_flag) {
    now_cnt = HAL_GetTick();
    elapsed = now_cnt - start_cnt;

    // ESP32への通信タイムアウト（ミリ秒単位。とりあえずTIMEOUT_CNTを流用または少し長めに設定可能。ここでは5msとする等でもよいが、既存のTIMEOUT_CNTを使用）
    if (elapsed > TIMEOUT_CNT) {
      HAL_UART_AbortReceive_IT(ESP32_uart_ch); // IT受信を強制停止
      UART4_flag = 1; // タイムアウトでもwhileを抜けるように強制フラグ立て
      ESP32_Failed_Connection = 1; // データは無効化する
      break;
    }

    // もし高速通信によるオーバーランエラー(ORE)が発生してHALドライバが沈黙した場合に備え、手動でフラグをクリアする
    if (__HAL_UART_GET_FLAG(ESP32_uart_ch, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG(ESP32_uart_ch);
      HAL_UART_AbortReceive_IT(ESP32_uart_ch); // 一度アボートして
      HAL_UART_Receive_IT(ESP32_uart_ch, &RB, 1); // 即座に受信再開
    }
  }

  if (UART4_flag && !ESP32_Failed_Connection) {
    ESP32_RX_Data.byte = RB;
  }
}
