/*
 * File:   robot.hpp
 * Author: taku_256
 *
 * Created on March 1, 2026, 9:30 AM
 */

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include "BNO055.hpp"
#include "GYRO.hpp"
#include "chassis.hpp"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern "C"
{
#include "UART.h"
}

extern bool is_role_changed;
extern RobotState comm_state;
extern RoleState my_role;

// STANDALONE時のみロール変更を許可する関数
void change_role(RoleState new_role);

void request_kick();

// chassis helper for omni drive
extern Chassis<Omni_4> omni;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

extern volatile uint16_t ADC_ch1;
extern volatile uint16_t ADC_ch2;
// runtime state
extern int ball_deg;
extern double ball_dis;

extern double keeper_vec[2];

extern int LineDepth;
extern int LineAngle;
extern int LineAngle_before;
extern int LineX;
extern int LineY;
extern int lineSideRight;
extern int lineSideBack;
extern int lineSideLeft;

extern bool lineAngel;
extern int goal[8];

extern uint16_t cnt;

extern EULAR e;

extern uint8_t enemyGoal_Angle_ins;
extern uint8_t enemyGoal_Width_ins;
extern uint8_t myGoal_Angle_ins;
extern uint8_t myGoal_Width_ins;
extern int16_t enemyGoal_Angle;
extern int16_t enemyGoal_Width;
extern int16_t myGoal_Angle;
extern int16_t myGoal_Width;
extern uint8_t enemyGoal_Angle_Range_ins;
extern uint8_t enemyGoal_Radius_ins;
extern uint8_t myGoal_Angle_Range_ins;
extern uint8_t myGoal_Radius_ins;
extern int16_t enemyGoal_Angle_Range;
extern int16_t enemyGoal_Radius;
extern int16_t myGoal_Angle_Range;
extern int16_t myGoal_Radius;
extern double right_goal_angle;
extern double left_goal_angle;

// Line / state flags
extern bool outside_of_Line;
extern bool out_of_Line;
extern bool Line_approached;
extern int LineAngle_returning;
extern int LineAngle_approached;
extern double LineAngle_diff;
extern double LineAngle_diff_before;

// Ball freeze flags
extern bool Ball_Freeze;
extern bool Ball_Freeze_First;

// Forward state variables
extern bool yel_or_blue_determinded;

extern bool holding_ball;
extern bool shooting;
extern bool softHold;
extern bool ball_approaching;

extern uint16_t ball_startedHolding_time;
extern uint16_t ball_beenHolding_time;
extern bool ball_counting_ballHoldtime;
extern uint16_t ball_startedReleasing_time;
extern bool ball_counting_ballReleasetime;
extern uint16_t kicking_start_time;
extern bool kicking_active;
extern uint16_t ball_dribbleAccerationBasetime;
extern uint16_t ball_dribbleAccerationtime;
extern bool ball_dribbleAcceration;
extern bool starting_dribbler;
extern bool ball_notfound;
extern uint16_t ball_notfoundtime;
extern bool ball_outofreach;
extern uint16_t ball_outofreach_time;
extern bool takingBall_fromSide;

// Keeper state variables
extern int16_t mv_vector_x;
extern int16_t mv_vector_y;
extern const int16_t power;
extern int16_t ball_deg_Relative;

extern uint16_t Ball_Freeze_Start_CNT;
extern uint16_t Goal_Back_start_CNT;
extern const uint16_t GB_CNT;
extern const uint16_t GOAL_WIDTH_GB;
extern uint32_t Now_CNT;
extern const int16_t BF_Q_CNT;
extern const uint8_t Q_SIZE;
extern uint8_t Q_Index_Push;
extern uint8_t Q_Index_Pop;
extern int16_t mv_q[200];
extern uint16_t mv_q_sum;
extern const uint16_t BALL_FREEZE_TH;

// Forward small-state variables
extern int ball_deg_before;
extern double ball_dis_before;
extern bool side_of_Goal;
extern const uint8_t side_of_goal_thr;

// Forward holding vector
extern int16_t mv_vector_holding_x;
extern int16_t mv_vector_holding_y;

extern int16_t mv_deg;
extern int16_t mv_theta;
extern int16_t mv_power;

extern int GYRO_Z;

extern int16_t sideLine_x;
extern int16_t sideLine_y;
extern bool sideLine;

// UI state // flags
extern uint32_t tim3_halfDuty;
extern bool swRed;
extern bool pre_swRed;
extern bool swGreen;
extern bool pre_swGreen;
extern bool rotateMotor;
extern uint8_t i_spkr;

extern double dribbler_power;

// motorPWMs
extern uint32_t period_1;
extern uint32_t period_8;
extern uint32_t period_3;

extern bool use_buzzer_in_algo;

void setup();

void forward();

void keeper();

void Japan();

void BNO_init();

#endif /*ROBOT_HPP_*/

/*******************************************************************************
 End of File
*/
