/*
 * File:   robot.cpp
 * Author: taku_256
 *
 * Created on March 1, 2026, 9:30 AM
 */

#include "robot.hpp"

Chassis<Omni_4> omni;

volatile uint16_t ADC_ch1;
volatile uint16_t ADC_ch2;

int ball_deg;
double ball_dis;
int LineDepth;
int LineAngle;
int LineX;
int LineY;
int lineSideRight;
int lineSideBack;
int lineSideLeft;
bool lineAngel;
int goal[8];
uint16_t cnt;

// Ball freeze flags (moved from keeper)
bool Ball_Freeze = false;
bool Ball_Freeze_First = false;

// Forward state variables (moved from forward())
bool yel_or_blue_determinded = false;

bool holding_ball = false;
bool shooting = false;
bool softHold = false;
bool ball_approaching = false;

uint16_t ball_startedHolding_time = 0;
uint16_t ball_beenHolding_time = 0;
bool ball_counting_ballHoldtime = false;
uint16_t ball_startedReleasing_time = 0;
bool ball_counting_ballReleasetime = false;
uint16_t kicking_start_time = 0;
bool kicking_active = false;
uint16_t ball_dribbleAccerationBasetime = 0;
uint16_t ball_dribbleAccerationtime = 0;
bool ball_dribbleAcceration = false;
bool starting_dribbler = false;
uint16_t motor_current_abs_adc[4] = {};
uint16_t motor_current_average_abs_adc = 0;
bool motor_current_valid = false;
bool ball_notfound = false;
uint16_t ball_notfoundtime = 0;
bool ball_outofreach = false;
uint16_t ball_outofreach_time = 0;
bool takingBall_fromSide = false;

// Keeper state variables
int16_t mv_vector_x = 0;
int16_t mv_vector_y = 0;
const int16_t power = 60;
int16_t ball_deg_Relative = 0;

uint16_t Ball_Freeze_Start_CNT = 0;
uint16_t Goal_Back_start_CNT = 0;
const uint16_t GB_CNT = 500;
const uint16_t GOAL_WIDTH_GB = 130;
uint32_t Now_CNT = 0;
const int16_t BF_Q_CNT = 20;
const uint8_t Q_SIZE = 200;
uint8_t Q_Index_Push = 0;
uint8_t Q_Index_Pop = 0;
int16_t mv_q[200] = {};
uint16_t mv_q_sum = 0;
const uint16_t BALL_FREEZE_TH = 6500;

// Forward small-state variables
int ball_deg_before = 0;
double ball_dis_before = 0;
bool side_of_Goal = false;
const uint8_t side_of_goal_thr = 65;

// Forward holding vector
int16_t mv_vector_holding_x = 0;
int16_t mv_vector_holding_y = 0;

uint8_t enemyGoal_Angle_ins, enemyGoal_Width_ins, myGoal_Angle_ins,
    myGoal_Width_ins, enemyGoal_Angle_Range_ins, enemyGoal_Radius_ins,
    myGoal_Angle_Range_ins, myGoal_Radius_ins;
int16_t enemyGoal_Angle, enemyGoal_Width, myGoal_Angle, myGoal_Width,
    enemyGoal_Angle_Range, enemyGoal_Radius, myGoal_Angle_Range,
    myGoal_Radius;

double right_goal_angle = 0;
double left_goal_angle = 0;
double enemy_right_goal_angle = 0;
double enemy_left_goal_angle = 0;
double enemy_rightmiddle_goal_angle = 0;
double enemy_leftmiddle_goal_angle = 0;

int16_t mv_deg = 0;
int16_t mv_theta = 0;
int16_t mv_power = 0;

int GYRO_Z = 0;

int16_t sideLine_x;
int16_t sideLine_y;
bool sideLine;


// UI state // flags
uint32_t tim3_halfDuty = 0;
bool swRed = 0;
bool pre_swRed = 0;
bool swGreen = 0;
bool pre_swGreen = 0;
bool play = false;
bool rotateMotor = 0;
uint8_t i_spkr = 0;

double dribbler_power = 0;

// motor PWMs
uint32_t period_1;
uint32_t period_8;
uint32_t period_3;

void setup() {
  // PWM setup
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // A
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // B
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // C
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // speaker
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1); // dribbler input 2
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1); // dribbler input 1
  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);

  // JYRO setup
  HAL_Delay(500);
  BNO_init();

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start(&htim5);  // IR
  HAL_TIM_Base_Start(&htim7);

  HAL_UART_Receive_IT(&huart2, &Goal_RB, 1);
}

/*******************************************************************************
 End of File
*/
