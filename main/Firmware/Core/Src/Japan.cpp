// Japan.cpp – メインルーチン
#include "robot.hpp"
#include <algorithm>
using namespace std;

/*----------------------------*/
/*--- 書き込み時に必ず確認！！ ---*/
/*----------------------------*/
/*----------------------------*/
/*--- 書き込み時に必ず確認！！ ---*/
/*----------------------------*/
/*
 *-my_default_roleについて
  書き込み先ロボットの初期の役割です。
  ROLE_KEEPER or ROLE_FORWARD が利用可能です。
  TODO:書き込み時にこれを確認する、

 */

#define my_default_role ROLE_FORWARD

/*----------------------------*/
/*--- 書き込み時に必ず確認！！ ---*/
/*----------------------------*/
/*----------------------------*/
/*--- 書き込み時に必ず確認！！ ---*/
/*----------------------------*/

#define blueGoalAngle 0
#define blueGoalWidth 1
#define yelGoalAngle 2
#define yelGoalWidth 3
#define blueGoalAngleRange 4
#define blueGoalRadius 5
#define yelGoalAngleRange 6
#define yelGoalRadius 7

volatile uint32_t japan_control_period_cycles = 0;
volatile uint32_t japan_control_period_us = 0;
volatile uint32_t japan_control_period_min_us = 0xFFFFFFFFU;
volatile uint32_t japan_control_period_max_us = 0;
volatile uint32_t japan_control_loop_count = 0;

namespace
{
constexpr uint16_t DRIBBLER_PWM_MAX = 1000;
constexpr uint16_t DRIBBLER_ACTIVE_MAX = 990;
constexpr int16_t DRIBBLER_POWER_LIMIT = 999;
constexpr double DRIBBLER_DEFAULT_POWER = 990.0;
constexpr bool DEBUG_FORCE_DRIBBLER_REVERSE = false;
constexpr double DEBUG_DRIBBLER_REVERSE_POWER = -400.0;
constexpr uint16_t KICK_HOLD_COUNT = 300;
constexpr uint16_t KICK_INTERVAL_COUNT = 2000;
constexpr uint8_t ADC_FILTER_SHIFT = 5;

bool kick_interval_active = false;
uint16_t kick_interval_start_time = 0;
int32_t adc_ch1_filter_accum = 0;
int32_t adc_ch2_filter_accum = 0;
bool adc_ch1_filter_initialized = false;
bool adc_ch2_filter_initialized = false;
uint32_t control_period_prev_cycle = 0;

void init_control_period_measurement()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  control_period_prev_cycle = DWT->CYCCNT;
  japan_control_period_cycles = 0U;
  japan_control_period_us = 0U;
  japan_control_period_min_us = 0xFFFFFFFFU;
  japan_control_period_max_us = 0U;
  japan_control_loop_count = 0U;
}

void update_control_period_measurement()
{
  const uint32_t now_cycle = DWT->CYCCNT;
  const uint32_t elapsed_cycles = now_cycle - control_period_prev_cycle;
  control_period_prev_cycle = now_cycle;

  if (japan_control_loop_count != 0U)
  {
    const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    const uint32_t elapsed_us =
        cycles_per_us == 0U ? 0U : elapsed_cycles / cycles_per_us;

    japan_control_period_cycles = elapsed_cycles;
    japan_control_period_us = elapsed_us;

    if (elapsed_us < japan_control_period_min_us)
    {
      japan_control_period_min_us = elapsed_us;
    }
    if (elapsed_us > japan_control_period_max_us)
    {
      japan_control_period_max_us = elapsed_us;
    }
  }

  ++japan_control_loop_count;
}

uint16_t low_pass_adc(uint16_t raw, int32_t &accum, bool &initialized)
{
  if (!initialized)
  {
    accum = static_cast<int32_t>(raw) << ADC_FILTER_SHIFT;
    initialized = true;
    return raw;
  }

  accum += static_cast<int32_t>(raw) - (accum >> ADC_FILTER_SHIFT);
  return static_cast<uint16_t>(accum >> ADC_FILTER_SHIFT);
}

uint32_t dribbler_compare(TIM_HandleTypeDef *htim, uint16_t duty)
{
  if (duty > DRIBBLER_PWM_MAX)
  {
    duty = DRIBBLER_PWM_MAX;
  }
  if (duty == DRIBBLER_PWM_MAX)
  {
    duty = DRIBBLER_ACTIVE_MAX;
  }

  const uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
  return ((period * duty) + (DRIBBLER_PWM_MAX / 2U)) / DRIBBLER_PWM_MAX;
}

void set_dribbler_pwm(uint16_t drb1_duty, uint16_t drb2_duty)
{
  static bool initialized = false;
  static uint16_t last_drb1_duty = 0;
  static uint16_t last_drb2_duty = 0;

  if (drb1_duty > DRIBBLER_PWM_MAX)
  {
    drb1_duty = DRIBBLER_PWM_MAX;
  }
  if (drb2_duty > DRIBBLER_PWM_MAX)
  {
    drb2_duty = DRIBBLER_PWM_MAX;
  }

  const bool reverse_change =
      initialized &&
      ((last_drb1_duty > 0 && drb2_duty > 0) ||
       (last_drb2_duty > 0 && drb1_duty > 0));

  if (reverse_change)
  {
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
    for (volatile uint32_t i = 0; i < 1000U; ++i)
    {
      __NOP();
    }
  }

  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1,
                        dribbler_compare(&htim13, drb1_duty));
  __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1,
                        dribbler_compare(&htim14, drb2_duty));

  last_drb1_duty = drb1_duty;
  last_drb2_duty = drb2_duty;
  initialized = true;
}

double clamp_dribbler_power(double power)
{
  if (power > DRIBBLER_POWER_LIMIT)
  {
    return DRIBBLER_POWER_LIMIT;
  }
  if (power < -DRIBBLER_POWER_LIMIT)
  {
    return -DRIBBLER_POWER_LIMIT;
  }
  return power;
}

uint16_t dribbler_duty_from_power(double power)
{
  const double magnitude = power < 0.0 ? -power : power;
  return static_cast<uint16_t>(
      (magnitude * DRIBBLER_ACTIVE_MAX / DRIBBLER_POWER_LIMIT) + 0.5);
}

void apply_dribbler_power(bool force_stop)
{
  if (force_stop)
  {
    set_dribbler_pwm(0, 0);
    return;
  }

  dribbler_power = clamp_dribbler_power(dribbler_power);
  const uint16_t duty = dribbler_duty_from_power(dribbler_power);

  if (duty == 0)
  {
    set_dribbler_pwm(0, 0);
  }
  else if (dribbler_power > 0.0)
  {
    set_dribbler_pwm(0, duty);
  }
  else
  {
    set_dribbler_pwm(duty, 0);
  }
}

bool kick_interval_is_active()
{
  if (!kick_interval_active)
  {
    return false;
  }

  if ((uint16_t)(cnt - kick_interval_start_time) > KICK_INTERVAL_COUNT)
  {
    kick_interval_active = false;
    return false;
  }

  return true;
}

bool update_kicking_hysteresis()
{
  if (kicking_active)
  {
    HAL_GPIO_WritePin(KICK2_GPIO_Port, KICK2_Pin, GPIO_PIN_SET);

    if ((uint16_t)(cnt - kicking_start_time) > KICK_HOLD_COUNT)
    {
      kicking_active = false;
      kick_interval_active = true;
      kick_interval_start_time = cnt;
      HAL_GPIO_WritePin(KICK2_GPIO_Port, KICK2_Pin, GPIO_PIN_RESET);
    }

    return true;
  }
  else
  {
    kick_interval_is_active();
    HAL_GPIO_WritePin(KICK2_GPIO_Port, KICK2_Pin, GPIO_PIN_RESET);
  }

  return false;
}
}

void request_kick()
{
  if (kicking_active || kick_interval_is_active())
  {
    return;
  }

  kicking_start_time = cnt;
  kicking_active = true;
}

// --- Role Management FSM ---
RobotState comm_state = STATE_STANDALONE;
RoleState my_role = my_default_role;
uint32_t last_valid_packet_time = 0;
uint32_t role_change_pending_time = 0;
uint32_t role_lock_time = 0;
bool is_role_change_pending = false;
RoleState pending_role = my_default_role;
bool first_run_fsm = true;
bool is_role_changed = false; // ロール変更通知フラグ
bool use_buzzer_in_algo = false;

void update_role_management() {
  uint32_t now = HAL_GetTick();
  bool partner_active = false;

  // 1. パケット有効性チェックと Mutual Confirmation
  if (!ESP32_Failed_Connection && ESP32_RX_Data.no_connection) {
    last_valid_packet_time = now;
    ESP32_TX_Data.local_ACK = 1;
  } else {
    ESP32_TX_Data.local_ACK = 0;
  }

  if ((now - last_valid_packet_time) > 5000) {
    if (comm_state == STATE_COORDINATED ||
        comm_state == STATE_RECOVERING) {
      // 通信が確立していた状態から切断された: 相手の電源が落ちたと判断
      ESP32_TX_Data.youWereDead = 1; // 「お前はすでに死んでいる」
    }
    comm_state = STATE_STANDALONE;
    partner_active = false;
  } else {
    partner_active = true;
  }

  bool mutual_confirmed = (partner_active && ESP32_RX_Data.local_ACK == 1 &&
                           ESP32_TX_Data.local_ACK == 1);

  if (mutual_confirmed) {
    if (comm_state == STATE_STANDALONE) {
      comm_state = STATE_RECOVERING;
    }
    // 相手からyouWereDeadを受けた場合もRECOVERINGに戻す
    // (COORDINATED中にchange_role()でロールを変えた直後に復帰した場合の競合解決)
    if (ESP32_RX_Data.youWereDead && comm_state == STATE_COORDINATED) {
      comm_state = STATE_RECOVERING;
    }
  } else {
    if (comm_state != STATE_STANDALONE) {
      comm_state = STATE_STANDALONE;
      ESP32_TX_Data.youWereDead = 1; // 相手が落ちたことを記録
    }
  }

  // 2. ロール管理 FSM
  RoleState target_role = my_role;

  if (comm_state == STATE_STANDALONE) {
    // STANDALONE時は現在のロールを維持（change_role()で手動変更可能）
    target_role = my_role;
  } else if (comm_state == STATE_RECOVERING) {
    // 復帰時: 現在のロールを維持しつつフラグをクリア
    target_role = my_role;
    ESP32_TX_Data.youWereDead = 0;

    // ロール競合チェック:
    // 両機体が同じロールの場合、my_default_roleから離れている方が譲歩
    if (my_role == ESP32_RX_Data.role) {
      if (my_role != my_default_role) {
        target_role = my_default_role;
      }
    }

    if (my_role == target_role) {
      comm_state = STATE_COORDINATED;
    }
  } else if (comm_state == STATE_COORDINATED) {
    // 重複の絶対回避
    if (my_role == ESP32_RX_Data.role) {
      // お互いが同じロールになってしまった場合、本来の役割(デフォルト)から離れている方が譲歩(退避)する
      if (my_role != my_default_role) {
        target_role = my_default_role;
      } else {
        target_role = my_role;
      }
    }
  }

  // 3. チャタリング防止と安定性 (Hysteresis & Lock)
  if (target_role != my_role) {
    if ((uint16_t)(now - role_lock_time) < 1000) {
      is_role_change_pending = false;
    } else {
      if (!is_role_change_pending || pending_role != target_role) {
        is_role_change_pending = true;
        pending_role = target_role;
        role_change_pending_time = now;
      } else if (now - role_change_pending_time >= 200) {
        my_role = target_role;
        role_lock_time = now;
        is_role_change_pending = false;
        is_role_changed =
            true; // ロールが実際に切り替わった瞬間にフラグを立てる
      }
    }
  } else {
    is_role_change_pending = false;
  }

  ESP32_TX_Data.role = my_role;
}

// STANDALONE時のみロール変更を許可する関数
// forward.cpp / keeper.cpp のアルゴリズム内から呼び出し可能
void change_role(RoleState new_role) {
  if (comm_state == STATE_STANDALONE && my_role != new_role) {
    ESP32_TX_Data.youWereDead = 1; // 「お前はすでに死んでいる」
    my_role = new_role;
    ESP32_TX_Data.role = my_role;
    is_role_changed = true;
  }
}
void Japan()
{

  // ---- [Common Initialization] ----
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
  set_dribbler_pwm(0, 0);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_Delay(500);

  BNO_init();

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start(&htim5);  // IR
  HAL_TIM_Base_Start(&htim7);
  HAL_TIM_Base_Start_IT(&htim9); // ESP32 Communication interval

  HAL_UART_Receive_IT(&huart2, &Goal_RB, 1);

  while (!(Goal_ava == true))
  {
  }

  period_1 = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
  period_8 = __HAL_TIM_GET_AUTORELOAD(&htim8) + 1;
  period_3 = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;

  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);

//  HAL_Delay(700);

  goal[0] = (int)Goal_Blue_angle;
  goal[1] = Goal_Blue_size;
  goal[2] = (int)Goal_Yellow_angle;
  goal[3] = Goal_Yellow_size;
  goal[4] = (int)Goal_Blue_angle_range;
  goal[5] = (int)Goal_Blue_radius;
  goal[6] = (int)Goal_Yellow_angle_range;
  goal[7] = (int)Goal_Yellow_radius;

  if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13))
  {
    enemyGoal_Angle_ins = yelGoalAngle;
    enemyGoal_Width_ins = yelGoalWidth; // 黄色攻め 自陣青
    myGoal_Angle_ins = blueGoalAngle;
    myGoal_Width_ins = blueGoalWidth;
    enemyGoal_Angle_Range_ins = yelGoalAngleRange;
    enemyGoal_Radius_ins = yelGoalRadius;
    myGoal_Angle_Range_ins = blueGoalAngleRange;
    myGoal_Radius_ins = blueGoalRadius;

    while (i_spkr < 3) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      HAL_Delay(100);

      i_spkr++;
    }
  }
  else
  {
    enemyGoal_Angle_ins = blueGoalAngle;
    enemyGoal_Width_ins = blueGoalWidth; // 青攻め　自陣黄色
    myGoal_Angle_ins = yelGoalAngle;
    myGoal_Width_ins = yelGoalWidth;
    enemyGoal_Angle_Range_ins = blueGoalAngleRange;
    enemyGoal_Radius_ins = blueGoalRadius;
    myGoal_Angle_Range_ins = yelGoalAngleRange;
    myGoal_Radius_ins = yelGoalRadius;

    while (i_spkr < 2) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
      HAL_Delay(200);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      HAL_Delay(200);

      i_spkr++;
    }
  }

  HAL_Delay(500);

  init_control_period_measurement();

  while (1)
  {
    update_control_period_measurement();

    if (starting_dribbler == true)
    {
      //      __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNE/////////////p]NNEL_1, 500);
    }

    cnt = __HAL_TIM_GET_COUNTER(&htim7);

    mv_theta = 0;
    mv_power = 0;
    mv_deg = 0;
    mv_power = 90;

    goal[0] = (int)Goal_Blue_angle;
    goal[1] = Goal_Blue_size;
    goal[2] = (int)Goal_Yellow_angle;
    goal[3] = Goal_Yellow_size;
    goal[4] = (int)Goal_Blue_angle_range;
    goal[5] = (int)Goal_Blue_radius;
    goal[6] = (int)Goal_Yellow_angle_range;
    goal[7] = (int)Goal_Yellow_radius;

    get_IR(&huart5); // Global変数のBall_Theta, Ball_Closenessに格納
    get_LINE(&huart6);
    get_MAIN_SUB(&huart3);

    if (ESP32_Comm_flag)
    {
      ESP32_Comm_flag = 0;

      // 送信データのセット
      ESP32_TX_Data.hold_flag = 1;

      get_ESP32(&huart4, ESP32_TX_Data);
      update_role_management();
    }

    GYRO_Z = e.z;

    GYRO_AngleOffset = 0;

    ball_deg = Ball_Theta * -1;
    ball_dis = 115 - Ball_Closeness;
    LineX = LINE_X;
    LineY = LINE_Y;
    lineSideRight = LINE_Side_Right;
    lineSideBack = LINE_Side_Back;
    lineSideLeft = LINE_Side_Left;
    LineAngle = atan2(LineX, LineY) * 180 / M_PI;
    LineDepth = sqrt(LineY * LineY + LineX * LineX);
    lineAngel = LINE_Angel;

    enemyGoal_Angle = goal[enemyGoal_Angle_ins];
    enemyGoal_Width = goal[enemyGoal_Width_ins];
    myGoal_Angle = goal[myGoal_Angle_ins];
    myGoal_Width = goal[myGoal_Width_ins];

    enemyGoal_Angle_Range = goal[enemyGoal_Angle_Range_ins];
    enemyGoal_Radius = goal[enemyGoal_Radius_ins];
    myGoal_Angle_Range = goal[myGoal_Angle_Range_ins];
    myGoal_Radius = goal[myGoal_Radius_ins];

    right_goal_angle = myGoal_Angle - myGoal_Angle_Range / 2.0;
    left_goal_angle = myGoal_Angle + myGoal_Angle_Range / 2.0;
    if (left_goal_angle > 180) left_goal_angle -= 360;
    if (right_goal_angle < -180) right_goal_angle += 360;

    shooting = false;
    softHold = false;

    // ADC1(BALL1)read--------------------------------------
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
    {
      ADC_ch1 = low_pass_adc(static_cast<uint16_t>(HAL_ADC_GetValue(&hadc1)),
                             adc_ch1_filter_accum,
                             adc_ch1_filter_initialized);
    }
    HAL_ADC_Stop(&hadc1);
    //------------------------------------------------------

    // ADC2(BALL2)read--------------------------------------
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 1000) == HAL_OK)
    {
      ADC_ch2 = low_pass_adc(static_cast<uint16_t>(HAL_ADC_GetValue(&hadc2)),
                             adc_ch2_filter_accum,
                             adc_ch2_filter_initialized);
    }
    HAL_ADC_Stop(&hadc2);
    //------------------------------------------------------

    const bool holding_ball_allowed = !kicking_active && !kick_interval_is_active();

    if (!holding_ball_allowed) {
      holding_ball = false;
      ball_counting_ballHoldtime = false;
      ball_counting_ballReleasetime = false;
    } else if (ADC_ch2 > 600) {
      ball_counting_ballHoldtime =
          false; // Release判定に入ったらHoldタイマーをリセット
      if (!ball_counting_ballReleasetime) {
        ball_startedReleasing_time = cnt;
        ball_counting_ballReleasetime = true;
      } else {
        if ((uint16_t)(cnt - ball_startedReleasing_time) > 400) {
          holding_ball = false;
        }
      }
    } else if (ADC_ch2 < 350) {
      ball_counting_ballReleasetime =
          false; // Hold判定に入ったらReleaseタイマーをリセット
      if (!ball_counting_ballHoldtime) {
        ball_startedHolding_time = cnt;
        ball_counting_ballHoldtime = true;
      } else {
        if ((uint16_t)(cnt - ball_startedHolding_time) > 300) {
          holding_ball = true;
        }
      }
    }

//     if(kicking){
//    	 holding_bal = false;
//     }

    // --- Dynamic Algorithm Execution ---
    if (my_role == ROLE_KEEPER)
    {
      keeper();
    }
    else
    {
      forward();
    }

    const bool kick_hold_this_cycle = update_kicking_hysteresis();

    // --- Common Gyro & Output application ---

    // タイマー(htim7)のcntを用いて1ループあたりの時間(dt)を計算
    dt = (double)(uint16_t)(cnt - GYRO_precnt);
    if (dt <= 0.0)
      dt = 1.0; // ゼロ割防止

    if (!(GYRO_AngleOffset == 0))
    {
      GYRO_E = static_cast<int>(e.z) + GYRO_AngleOffset;
      GYRO_kp = 0.5;
    }
    else
    {
      GYRO_E = static_cast<int>(e.z);
    }

    // I項の計算
    GYRO_I += GYRO_E * dt;

    // D項含めPIDによるduty計算 (D項は時間変化で割る)
    GYRO_duty = (int)(GYRO_E * GYRO_kp + GYRO_I * GYRO_ki +
                      ((GYRO_E - GYRO_preE) / dt) * GYRO_kd);
    GYRO_duty = GYRO_duty * -18;

    if (GYRO_duty > 800)
    {
      GYRO_duty = 800;
    }
    else if (GYRO_duty < -800)
    {
      GYRO_duty = -800;
    }

    if (!(GYRO_preE == GYRO_E))
    {
      GYRO_preE = GYRO_E;
    }
    GYRO_precnt = cnt; // 今回のタイマー値を保存

    // compute and apply wheel outputs using chassis helper

    // Keep compare values inside the 0..ARR range when adding/subtracting
    // from the half-period center value.
    omni.set_limit((period_1 / 2) - 10);

//    mv_power = 100;

    omni.dcalc((mv_deg * -1) + 90, (mv_power* 900 / 100), GYRO_duty);
//    omni.dcalc(90, (mv_power * 0 / 100), 0); // デバッグ用そのまま

    swRed = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
    swGreen = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

    if (!use_buzzer_in_algo) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
    use_buzzer_in_algo = false;

    if (swRed == 1 && pre_swRed == 0)
    {
      rotateMotor = !rotateMotor;
    }

    if (rotateMotor)
    {
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
      if (DEBUG_FORCE_DRIBBLER_REVERSE)
      {
        dribbler_power = DEBUG_DRIBBLER_REVERSE_POWER;
        apply_dribbler_power(false);
      }
      else
      {
        if (!kick_hold_this_cycle)
        {
          dribbler_power = DRIBBLER_DEFAULT_POWER;
        }
        apply_dribbler_power(false);
      }

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    }
    else
    {
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
      set_dribbler_pwm(0, 0);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
    }

    pre_swRed = swRed;
    pre_swGreen = swGreen;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    // PWM order: front right -> back right -> back left -> front left.
    // omni index order: front left -> back left -> back right -> front right.


//    double a1 = *omni.get_motor(3);

    //     front right
    if (*(omni.get_motor(3)) > 0)
    {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
                            (period_8 / 2) + abs(*omni.get_motor(3)));
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
                            (period_8 / 2) - abs(*omni.get_motor(3)));
    }

    //     back right
    if (*(omni.get_motor(2)) > 0)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                            (period_1 / 2) + abs(*omni.get_motor(2)));
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                            (period_1 / 2) - abs(*omni.get_motor(2)));
    }

    //     back left
    if (*(omni.get_motor(1)) > 0)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                            (period_1 / 2) + abs(*omni.get_motor(1)));
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                            (period_1 / 2) - abs(*omni.get_motor(1)));
    }

    //     front left
    if (*(omni.get_motor(0)) < 0)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                            (period_1 / 2) + abs(*omni.get_motor(0)));
    }
    else
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                            (period_1 / 2) - abs(*omni.get_motor(0)));
    }
  }
}
