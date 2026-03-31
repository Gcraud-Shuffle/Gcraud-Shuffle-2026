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

void Japan() {

  // ---- [Common Initialization] ----
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1); // drib
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);  // speaker

  HAL_Delay(500);

  BNO_init();

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start(&htim5);  // IR
  HAL_TIM_Base_Start(&htim14); // LINE
  HAL_TIM_Base_Start(&htim7);
  HAL_TIM_Base_Start_IT(&htim9); // ESP32 Communication interval

  HAL_UART_Receive_IT(&huart2, &Goal_RB, 1);

  while (!(Goal_ava == true)) {
  }

  period_1 = __HAL_TIM_GET_AUTORELOAD(&htim1) + 1;
  period_8 = __HAL_TIM_GET_AUTORELOAD(&htim8) + 1;
  period_3 = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1;

  __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 500);

  //  HAL_Delay(700);

  goal[0] = (int)Goal_Blue_angle;
  goal[1] = Goal_Blue_size;
  goal[2] = (int)Goal_Yellow_angle;
  goal[3] = Goal_Yellow_size;
  goal[4] = (int)Goal_Blue_angle_range;
  goal[5] = (int)Goal_Blue_radius;
  goal[6] = (int)Goal_Yellow_angle_range;
  goal[7] = (int)Goal_Yellow_radius;

  if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)) {
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
  } else {
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

  while (1) {

    if (starting_dribbler == true) {
      //      __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNE/////////////p]NNEL_1,
      //      500);
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

    get_IR(&huart5, &htim5); // Global変数のBall_Theta, Ball_Closenessに格納
    get_LINE(&huart6, &htim14);

    if (ESP32_Comm_flag) {
      ESP32_Comm_flag = 0;

      // 送信データのセット
      ESP32_TX_Data.hold_flag = 1;

      get_ESP32(&huart4, ESP32_TX_Data);
      update_role_management();
    }

    GYRO_Z = e.z;

    GYRO_AngleOffset = 0;

    ball_deg = Ball_Theta * -1;
    ball_dis = 80 - Ball_Closeness;
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
    if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
      ADC_ch1 = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    //------------------------------------------------------

    // ADC2(BALL2)read--------------------------------------
    HAL_ADC_Start(&hadc2);
    if (HAL_ADC_PollForConversion(&hadc2, 1000) == HAL_OK) {
      ADC_ch2 = HAL_ADC_GetValue(&hadc2);
    }
    HAL_ADC_Stop(&hadc2);
    //------------------------------------------------------

    if (ADC_ch2 > 400) {
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
    } else if (ADC_ch2 < 200) {
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

    // --- Dynamic Algorithm Execution ---
    if (my_role == ROLE_KEEPER) {
      keeper();
    } else {
      forward();
    }

    // --- Common Gyro & Output application ---

    // タイマー(htim7)のcntを用いて1ループあたりの時間(dt)を計算
    dt = (double)(uint16_t)(cnt - GYRO_precnt);
    if (dt <= 0.0)
      dt = 1.0; // ゼロ割防止

    if (!(GYRO_AngleOffset == 0)) {
      GYRO_E = static_cast<int>(e.z) + GYRO_AngleOffset;
      GYRO_kp = 0.5;
    } else {
      GYRO_E = static_cast<int>(e.z);
    }

    // I項の計算
    GYRO_I += GYRO_E * dt;

    // D項含めPIDによるduty計算 (D項は時間変化で割る)
    GYRO_duty = (int)(GYRO_E * GYRO_kp + GYRO_I * GYRO_ki +
                      ((GYRO_E - GYRO_preE) / dt) * GYRO_kd);
    GYRO_duty = GYRO_duty * -18;

    if (GYRO_duty > 800) {
      GYRO_duty = 800;
    } else if (GYRO_duty < -800) {
      GYRO_duty = -800;
    }

    if (!(GYRO_preE == GYRO_E)) {
      GYRO_preE = GYRO_E;
    }
    GYRO_precnt = cnt; // 今回のタイマー値を保存

    // compute and apply wheel outputs using chassis helper

    omni.set_limit(period_1 / 2);

    omni.dcalc((mv_deg * -1) + 90, (mv_power * 900 / 100), GYRO_duty);
    //    omni.dcalc(90, (mv_power * 0 / 100), GYRO_duty); // デバッグ用そのまま

    swRed = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    swGreen = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);

    if (!use_buzzer_in_algo) {
      if (swRed | swGreen) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
      } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
      }
    }
    use_buzzer_in_algo = false;

    if (swRed == 1 && pre_swRed == 0) {
      rotateMotor = !rotateMotor;
    }

    HAL_GPIO_WritePin(KICK2_GPIO_Port, KICK2_Pin, GPIO_PIN_RESET);

    if (rotateMotor) {
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
      if (ball_dis == 80) {
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 500);
      } else {
        __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, dribbler_power + 500);
        dribbler_power = 130;
      }

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    } else {
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
      HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
      __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 500);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    }

    pre_swRed = swRed;
    pre_swGreen = swGreen;

    //     front left
    if (*(omni.get_motor(0)) > 0) {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
                            (period_1 / 2) + abs(*omni.get_motor(0)));
    } else {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
                            (period_1 / 2) - abs(*omni.get_motor(0)));
    }

    //     back left
    if (*(omni.get_motor(1)) > 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                            (period_1 / 2) + abs(*omni.get_motor(1)));
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                            (period_1 / 2) - abs(*omni.get_motor(1)));
    }

    //     back right
    if (*(omni.get_motor(2)) > 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                            (period_1 / 2) + abs(*omni.get_motor(2)));
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                            (period_1 / 2) - abs(*omni.get_motor(2)));
    }

    //     front right
    if (*(omni.get_motor(3)) > 0) {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                            (period_8 / 2) + abs(*omni.get_motor(3)));
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                            (period_8 / 2) - abs(*omni.get_motor(3)));
    }
  }
}
