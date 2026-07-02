// forward.cpp – 前方動作ルーチン
#include "robot.hpp"

#define blueGoalAngle 0
#define blueGoalWidth 1
#define yelGoalAngle 2
#define yelGoalWidth 3

// --- Holding Ball State Machine Definitions ---
enum class HoldState
{
  ACCELERATE,
  PULL_OUT,
  SHOOT_AIM,
  KICKING,
  ORBIT_AVOID,
  PUSH_OR_ESCAPE
};

enum class PushBallToGoalState
{
  PREPARE_WAITING,
  WAITING,
  WAITING_GRACE,
  APPROACHING,
  ESCAPING
};
PushBallToGoalState push_state = PushBallToGoalState::PREPARE_WAITING;
uint32_t push_wait_start_time = 0;
uint32_t push_approach_start_time = 0;
uint32_t push_escape_start_time = 0;
uint32_t push_line_escaped_time_when_waiting = 0;
uint32_t last_front_line_trace_time = 0;
double LineAngle_before_Approaching;

namespace
{
constexpr uint16_t kTimer7CountsPerMs = 2;
constexpr uint16_t kDefaultDribbleRampTimeMs = 1000;
constexpr int16_t kHoldingDribbleTargetPower = 50;
constexpr int16_t kLineDribbleTargetPower = 50;
constexpr uint8_t kMotorCurrentCount = 4;
constexpr uint16_t kCurrentAdcCenter = 2048;
constexpr int16_t kLineOverCenterEnterDeg = 130;
constexpr int16_t kLineOverCenterExitDeg = 70;
constexpr uint32_t kLineOverCenterMinHoldMs = 30;
constexpr uint32_t kLineOutReturnTimeoutMs = 300;
constexpr int16_t kLineOutReturnMinPower = 90;

struct LineOverrunState
{
  bool line_seen = false;
  bool over_center = false;
  bool returning_after_loss = false;
  int approach_angle = 0;
  int last_line_angle = 0;
  uint32_t over_center_start_ms = 0;
  uint32_t return_start_ms = 0;
};

LineOverrunState line_overrun;

bool &dribbleAccelerationActive()
{
  return ball_dribbleAcceration;
}

uint16_t &dribbleAccelerationBaseCnt()
{
  return ball_dribbleAccerationBasetime;
}

uint16_t &dribbleAccelerationElapsedMs()
{
  return ball_dribbleAccerationtime;
}

uint16_t timer7CountsToMs(uint16_t counts)
{
  return counts / kTimer7CountsPerMs;
}

void updateDribbleAccelerationRamp(bool holding, uint16_t nowCnt)
{
  if (!holding)
  {
    dribbleAccelerationActive() = false;
    dribbleAccelerationElapsedMs() = 0;
    return;
  }

  if (!dribbleAccelerationActive())
  {
    dribbleAccelerationActive() = true;
    dribbleAccelerationBaseCnt() = nowCnt;
    dribbleAccelerationElapsedMs() = 0;
    return;
  }

  const uint16_t elapsedCounts =
      static_cast<uint16_t>(nowCnt - dribbleAccelerationBaseCnt());
  dribbleAccelerationElapsedMs() = timer7CountsToMs(elapsedCounts);
}

int16_t linearRampPower(uint16_t elapsedMs, int16_t targetPower,
                        uint16_t rampTimeMs)
{
  if (rampTimeMs == 0 || elapsedMs >= rampTimeMs)
  {
    return targetPower;
  }

  return static_cast<int16_t>(
      (static_cast<int32_t>(targetPower) * elapsedMs) / rampTimeMs);
}

int16_t holdingDribbleRampPower()
{
  return linearRampPower(dribbleAccelerationElapsedMs(),
                         kHoldingDribbleTargetPower,
                         kDefaultDribbleRampTimeMs);
}

int16_t lineDribbleRampPower()
{
  return linearRampPower(dribbleAccelerationElapsedMs(),
                         kLineDribbleTargetPower,
                         kDefaultDribbleRampTimeMs);
}

uint16_t currentAbsFromAdc(uint16_t rawAdc)
{
  if (rawAdc >= kCurrentAdcCenter)
  {
    return static_cast<uint16_t>(rawAdc - kCurrentAdcCenter);
  }

  return static_cast<uint16_t>(kCurrentAdcCenter - rawAdc);
}

void updateMotorCurrentSense()
{
  uint32_t totalCurrent = 0;

  motor_current_valid =
      (MainSub_Status & MAIN_SUB_STATUS_CURRENT_VALID) != 0U;

  for (uint8_t index = 0; index < kMotorCurrentCount; index++)
  {
    motor_current_abs_adc[index] =
        currentAbsFromAdc(MainSub_Current_ADC12[index]);
    totalCurrent += motor_current_abs_adc[index];
  }

  motor_current_average_abs_adc =
      static_cast<uint16_t>(totalCurrent / kMotorCurrentCount);
}

int normalizeAngleDiff(int diff)
{
  while (diff > 180)
  {
    diff -= 360;
  }

  while (diff < -180)
  {
    diff += 360;
  }

  return diff;
}

int angleDiffAbs(int fromAngle, int toAngle)
{
  const int diff = normalizeAngleDiff(fromAngle - toAngle);
  return diff < 0 ? -diff : diff;
}

void resetLineOverrunState()
{
  line_overrun.line_seen = false;
  line_overrun.over_center = false;
  line_overrun.returning_after_loss = false;
  line_overrun.over_center_start_ms = 0;
}

void updateLineOverrunState(uint32_t nowMs)
{
  if (lineAngel)
  {
    if (!line_overrun.line_seen && !line_overrun.returning_after_loss)
    {
      line_overrun.approach_angle = LineAngle;
      line_overrun.over_center = false;
    }

    line_overrun.line_seen = true;
    line_overrun.returning_after_loss = false;
    line_overrun.last_line_angle = LineAngle;

    const int angle_diff =
        angleDiffAbs(line_overrun.approach_angle, LineAngle);

    if (line_overrun.over_center)
    {
      if (angle_diff <= kLineOverCenterExitDeg)
      {
        line_overrun.over_center = false;
        line_overrun.over_center_start_ms = 0;
      }
    }
    else if (angle_diff >= kLineOverCenterEnterDeg)
    {
      line_overrun.over_center = true;
      line_overrun.over_center_start_ms = nowMs;
    }

    return;
  }

  if (line_overrun.line_seen && line_overrun.over_center)
  {
    const bool over_center_stable =
        (nowMs - line_overrun.over_center_start_ms) >=
        kLineOverCenterMinHoldMs;

    if (!over_center_stable)
    {
      resetLineOverrunState();
      return;
    }

    line_overrun.line_seen = false;
    line_overrun.returning_after_loss = true;
    line_overrun.return_start_ms = nowMs;
    return;
  }

  if (line_overrun.returning_after_loss)
  {
    if ((nowMs - line_overrun.return_start_ms) > kLineOutReturnTimeoutMs)
    {
      resetLineOverrunState();
    }
    return;
  }

  resetLineOverrunState();
}

void keepLineReturnPower()
{
  if (mv_power < kLineOutReturnMinPower)
  {
    mv_power = kLineOutReturnMinPower;
  }
}

void applyLineOverCenterCorrection()
{
  if (lineAngel && line_overrun.over_center)
  {
    mv_deg = LineAngle;
    keepLineReturnPower();
  }
}

void applyLineLossReturnCorrection()
{
  if (line_overrun.returning_after_loss)
  {
    mv_deg = line_overrun.last_line_angle;
    keepLineReturnPower();
  }
}
}

// 敵回避用のベクトル計算関数
inline void calculateOrbitVector(double goalAngle, double goalWidth,
                                 double sideAngleThr, double &outY,
                                 double &outX)
{
  if (sideAngleThr == 0)
  {
    sideAngleThr = 65; // ゼロ割り防止
  }

  if (goalAngle > 0)
  {
    outX = (100.0 * cos(goalAngle * (90.0 / sideAngleThr) * (M_PI / 180.0)));
    outY = (-100.0 * sin(goalAngle * (M_PI / 180.0))) + (enemyGoal_Width * 1);
  }
  else
  {
    outX = (-100.0 * cos(goalAngle * (90.0 / sideAngleThr) * (M_PI / 180.0)));
    outY = (100.0 * sin(goalAngle * (M_PI / 180.0))) + (enemyGoal_Width * 1);
  }
}

HoldState current_state = HoldState::ORBIT_AVOID;

void forward()
{
  // `forward()` 全体で参照できるようにここで宣言しておく

  updateMotorCurrentSense();

	mv_power = 85;
  if (holding_ball)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  }

  //	holding_ball = true;
  //
  //	if(softHold == false && holding_ball == true){
  //		softHold = true;
  //	}

  if (ball_deg >= 0)
  {
    if (ball_deg <= 2)
    {
      mv_deg = 0;
    }
    else if (ball_deg <= 30)
    {
      //	           mv_deg = ball_deg*ball_deg / 10;
      mv_deg = ball_deg * 2.6;
    }
    else if (ball_deg <= 90)
    {
      mv_deg = ball_deg * 2.4;
      //			  mv_deg = ball_deg * 1.5;
      // mv_deg = ball_deg*ball_deg / 45;
    }
    else if (ball_deg <= 150)
    {
      mv_deg = ball_deg * 1.7;
    }
    else
    {
      mv_deg = ball_deg + 90;
    }
  }

  if (ball_deg <= 0)
  {
    if (ball_deg >= -2)
    {
      mv_deg = 0;
    }
    else if (ball_deg >= -30)
    {
      //	           mv_deg = (ball_deg*ball_deg / 10)*-1;
      mv_deg = ball_deg * 2.6;
    }
    else if (ball_deg >= -90)
    {
      mv_deg = ball_deg * 2.4;
      //			  mv_deg = ball_deg * 1.5;
      // mv_deg = (ball_deg*ball_deg / 45)*-1;
    }
    else if (ball_deg >= -150)
    {
      mv_deg = ball_deg * 1.7;
    }
    else
    {
      mv_deg = ball_deg + -90;
    }
  }

  //    if (ball_deg >= 0) {
  //      if (ball_deg <= 2) {
  //        mv_deg = 0;
  //      } else if (ball_deg <= 30) {
  //        //	           mv_deg = ball_deg*ball_deg / 10;
  //        mv_deg = ball_deg * 3.0;
  //      } else if (ball_deg <= 100) {
  //        mv_deg = ball_deg + 90;
  //        //			  mv_deg = ball_deg * 1.5;
  //        // mv_deg = ball_deg*ball_deg / 45;
  //      } else if (ball_deg <= 150) {
  //        mv_deg = ball_deg + 90;
  //      } else {
  //        mv_deg = ball_deg + 90;
  //      }
  //    }
  //
  //    if (ball_deg <= 0) {
  //      if (ball_deg >= -2) {
  //        mv_deg = 0;
  //      } else if (ball_deg >= -30) {
  //        //	           mv_deg = (ball_deg*ball_deg / 10)*-1;
  //        mv_deg = ball_deg * 3.0;
  //      } else if (ball_deg >= -100) {
  //        mv_deg = ball_deg + -90;
  //        //			  mv_deg = ball_deg * 1.5;
  //        // mv_deg = (ball_deg*ball_deg / 45)*-1;
  //      } else if (ball_deg >= -150) {
  //        mv_deg = ball_deg + -90;
  //      } else {
  //        mv_deg = ball_deg + -90;
  //      }
  //    }

  if (ball_dis > 0)
  { // 距離による回り込み角度ゲイン減衰
    if (ball_deg > 0)
    {
      mv_deg -= (abs(mv_deg - ball_deg) * ball_dis / 70);
    }
    else
    {
      mv_deg += (abs(mv_deg - ball_deg) * ball_dis / 70);
    }
  }
//
//  if (ball_dis < 15 && abs(ball_deg) < 20)
//  {
//    mv_power = 85;
//  }

  if(abs(enemyGoal_Angle) < 55 && abs(ball_deg) < 60 && (MainSub_SafeUltrasonic_mm[0] > 400 && MainSub_SafeUltrasonic_mm[1] > 400) && !lineAngel){
	  if(enemyGoal_Angle > 0){
		GYRO_AngleOffset = enemy_rightmiddle_goal_angle;
	}else{
		GYRO_AngleOffset = enemy_leftmiddle_goal_angle;
	}
  }

//  	mv_deg = 0;
//    	holding_ball = true;

  updateDribbleAccelerationRamp(holding_ball, cnt);
  const bool any_side_line_active =
      (lineSideRight > 0) || (lineSideBack > 0) || (lineSideLeft > 0);
  const bool any_line_sensor_active = lineAngel || any_side_line_active;

//  holding_ball = true;

  if (holding_ball == true && abs(ball_deg) < 30)
  {
    starting_dribbler = true;

    // --- ステートの決定（状態の評価） ---
    if ((enemyGoal_Width > 60 && abs(enemyGoal_Angle) < 65) || ((MainSub_SafeUltrasonic_mm[0] < 400 || MainSub_SafeUltrasonic_mm[1] < 400) && abs(enemyGoal_Angle) < 45))
    {
      // ゴール前近めでシュート可能
      const bool gyro_aligned_to_goal = (abs(GYRO_Z + enemyGoal_Angle) < 8);
      const bool wide_goal = (enemyGoal_Width > 70);
      if ((gyro_aligned_to_goal) || (any_line_sensor_active && abs(LineAngle) < 45 && wide_goal))
      {
        current_state = HoldState::KICKING;
      }
      else
      {
        current_state = HoldState::SHOOT_AIM;
      }
    }
    else if ((myGoal_Width > 40 || enemyGoal_Width == 0))
    {
      // 自陣ゴールが近い（または相手ロボットと押し合っている）場合
      current_state = HoldState::PUSH_OR_ESCAPE;
    }
    else if (abs(enemyGoal_Angle) > side_of_goal_thr || (enemyGoal_Width == 0 && MainSub_SafeUltrasonic_mm[2] < 400))
    {
      // ゴール横の場合
      side_of_Goal = true;
      current_state = HoldState::PULL_OUT;
    }
    else
    {
      // デフォルトは敵回避の回り込み
      current_state = HoldState::ORBIT_AVOID;
    }

    //    current_state = HoldState::ORBIT_AVOID;

    // --- ステートごとの出力計算（モーター/ドリブラー制御） ---
    switch (current_state)
    {
    case HoldState::KICKING:
      // キック時は進行停止・ドリブラー停止
    	if(enemyGoal_Angle > 0){
			GYRO_AngleOffset = enemy_rightmiddle_goal_angle;
			mv_deg = enemy_rightmiddle_goal_angle;
		}else{
			GYRO_AngleOffset = enemy_leftmiddle_goal_angle;
			mv_deg = enemy_leftmiddle_goal_angle;
		}
      mv_power = 100;
      // GYRO_kp = 0.8;
      dribbler_power = -999; // Reverse dribbler while kicking.
      request_kick();
      break;

    case HoldState::SHOOT_AIM:
      // ゴール方向へ姿勢制御しつつ前進
    	if(enemyGoal_Angle > 0){
			GYRO_AngleOffset = enemy_rightmiddle_goal_angle;
			mv_deg = enemy_rightmiddle_goal_angle;
		}else{
			GYRO_AngleOffset = enemy_leftmiddle_goal_angle;
			mv_deg = enemy_leftmiddle_goal_angle;
		}

      mv_power = 100;
      // GYRO_kp = 0.8;
      break;

    case HoldState::PUSH_OR_ESCAPE:
      // 自陣から逃げるため前進 または 押し合い負け回避
      mv_power = 100;
      mv_deg = 0;
      break;

    case HoldState::PULL_OUT:
      // ゴール横から引き出すため後退
      mv_deg = 180;
      mv_power = holdingDribbleRampPower();
      break;

    case HoldState::ORBIT_AVOID:
    default:
    {
      double mv_holding_x, mv_holding_y;
      calculateOrbitVector(enemyGoal_Angle, enemyGoal_Width, side_of_goal_thr,
                           mv_holding_y, mv_holding_x);
      mv_deg = atan2(mv_holding_x, mv_holding_y) * (180.0 / M_PI);

      mv_power = holdingDribbleRampPower();
      break;
    }
    }
  }

//  return;

  // キック判定（shooting変数を介したロジック）をステートマシン内に統合したため削除
  if (Ball_Closeness == 0)
  {
    if (ball_notfound == false)
    {
      ball_notfound = true;
      ball_notfoundtime = cnt;
    }
    else
    {
      if ((uint16_t)(cnt - ball_notfoundtime) > 6000)
      {
        mv_deg = 180;
        mv_power = 70;
      }
      else
      {
        mv_deg = ball_deg_before + GYRO_E;
      }
    }
    dribbler_power = 0;
  }
  else
  {
    ball_notfound = false;
    ball_deg_before = ball_deg;
    ball_dis_before = ball_dis;
  }

  //	    mv_power = 0;

  //		mv_power = 0;

  sideLine_x = 0;
  sideLine_y = 0;
  sideLine = false;

//    if (lineSideRight > 0) {
//      if (lineSideRight < 3 && (ball_deg > 0 && ball_deg < 180)) {
//        if ((ball_deg * enemyGoal_Angle) > 0 && ball_deg > 55 &&
//            abs(enemyGoal_Angle) > 45) {
//          mv_deg = 180;
//        } else if ((ball_deg * myGoal_Angle) > 0 && abs(myGoal_Angle) < 135) {
//          mv_deg = 0;
//        } else {
//          if (ball_deg < 100) {
//            mv_power = cos((ball_deg - GYRO_AngleOffset + 50) * M_PI / 180) *
//                       mv_power * 1.5;
//            mv_deg = 0;
//            //			    mv_power = 60;
//            if (ball_outofreach == false) {
//              ball_outofreach = true;
//              ball_outofreach_time = cnt;
//            }
//          } else {
//            mv_deg = -90;
//          }
//        }
//      } else {
//        mv_deg = -90;
//      }
//    } else {
//    }
  if (!lineAngel && lineSideRight > 0 && !holding_ball)
  {
    if (lineSideRight < 3 && (ball_deg > 0 && ball_deg < 100))
    {
      mv_power =
          cos((ball_deg - GYRO_AngleOffset + 50) * M_PI / 180) * mv_power * 1.5;
      mv_deg = 0;
      //			    mv_power = 60;
      if (ball_outofreach == false)
      {
        ball_outofreach = true;
        ball_outofreach_time = cnt;
      }
    }
    else
    {
      mv_deg = -90;
    }
  }
  else
  {
  }

  if (!lineAngel && lineSideLeft > 0 && !holding_ball)
  {
    if (lineSideLeft < 3 && (ball_deg < 0 && ball_deg > -100))
    {
      mv_power =
          cos((ball_deg - GYRO_AngleOffset - 50) * M_PI / 180) * mv_power * 1.5;
      mv_deg = 0;
      //				mv_power = 60;
      if (ball_outofreach == false)
      {
        ball_outofreach = true;
        ball_outofreach_time = cnt;
      }
    }
    else
    {
      mv_deg = 90;
    }
  }
  else
  {
  }

//  if (lineSideBack > 0)
//  {
//    mv_deg = 0;
//    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    //			mv_power = sin(ball_deg * M_PI/180)*mv_power
//    //* 1.5;
//  }

  if ((uint16_t)(cnt - ball_outofreach_time) > 3000 && holding_ball == false)
  {
    if ((uint16_t)(cnt - ball_outofreach_time) < 6000)
    {
      takingBall_fromSide = true;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
      if (abs(ball_deg - static_cast<int>(e.z)) < 60)
      {
        GYRO_AngleOffset = ball_deg - static_cast<int>(e.z);
      }
      else
      {
        GYRO_AngleOffset = 0;
      }
    }
    else
    {
      takingBall_fromSide = false;
      ball_outofreach = false;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }
  }

  updateLineOverrunState(HAL_GetTick());

  // lineAngel is handled only while the sensor currently sees the line.
  if (lineAngel == true)
  {
    use_buzzer_in_algo = true;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);

    mv_deg = LineAngle + 180;
    mv_power = 200;

    if ((lineSideLeft < 3 && lineSideRight < 3) && abs(ball_deg) < 45 &&
        abs(enemyGoal_Angle) > 55 && holding_ball == false &&
        abs(LineAngle) < 30)
    { // かめら角とボール角とサイドラインのじょうけんにする
    	if(LineDepth < 100){
    		mv_deg = LineAngle + 180;
    	}else{
    		mv_power = sin(ball_deg * M_PI / 180) * 80 * 1.5;
		    mv_deg = 90;
    	}
    }
    else
    {
      if (dribbleAccelerationActive())
      {
    	  if(lineAngel && LineDepth < 100){
    		  mv_deg = LineAngle + 180;
    	  }else{
    		  mv_power = lineDribbleRampPower();
    	  }

      }
      else
      {
        mv_power = 90;
      }
    }
  }

//  if (holding_ball && lineAngel && LineDepth > 70 &&
//      (current_state == HoldState::PULL_OUT || current_state == HoldState::ORBIT_AVOID) &&
//      (LineAngle * enemyGoal_Angle > 0) && (abs(LineAngle) > 25))
//  {
//     mv_power = cos((mv_deg) * M_PI / 180) * mv_power;
//    if (enemyGoal_Angle > 0)
//    {
//      mv_deg = LineAngle - 90;
//    }
//    else
//    {
//      mv_deg = LineAngle + 90;
//    }
//  }

//	 --------------------------------------- //
//	 Section: 自陣側のライン処理
//	 --------------------------------------- //
//
//   自陣側の後ろにボールがある時のラインの処理
//   後ろだけ反応して、後ろにボールがある時のライントレース
  if(lineSideBack&&!lineAngel&&!lineSideLeft&&!lineSideRight&&abs(ball_deg) > 90){
    double out_vec[2] = {sin(ball_deg * M_PI / 180.0)*20*(4-lineSideBack), (lineSideBack-1)*30};
		mv_deg = atan2(out_vec[0], out_vec[1]) * (180.0 / M_PI);
		mv_power = sqrt(out_vec[0] * out_vec[0] + out_vec[1] * out_vec[1]);
  }
  // 自陣側のフィールドの外側にある時(右)
  else if(!lineAngel&&lineSideBack&&!lineSideLeft&&lineSideRight&&(ball_deg > 75||ball_deg < -150)&&(160 > left_goal_angle||myGoal_Width==0)){
    double out_vec[2] = {-(lineSideRight-1)*40, (lineSideBack-1)*40};
		mv_deg = atan2(out_vec[0], out_vec[1]) * (180.0 / M_PI);
		mv_power = sqrt(out_vec[0] * out_vec[0] + out_vec[1] * out_vec[1]);
  }
  // 自陣側のフィールドの外側にある時(左)
  else if(!lineAngel&&lineSideBack&&lineSideLeft&&!lineSideRight&&(ball_deg < -75||ball_deg > 150)&&(-160 < right_goal_angle||myGoal_Width==0)){
    double out_vec[2] = {(lineSideLeft-1)*40, (lineSideBack-1)*40};
		mv_deg = atan2(out_vec[0], out_vec[1]) * (180.0 / M_PI);
		mv_power = sqrt(out_vec[0] * out_vec[0] + out_vec[1] * out_vec[1]);
  }

//	// --------------------------------------- //
//	// Section: 自陣側のゴール前でのライン処理
//	// --------------------------------------- //
  uint32_t now_cnt = HAL_GetTick();
//  // ゴール前のラインの動き
//
//  // ゴール前
//  // TODO: ボールとの距離でライントレースするかを決めるようにする
  if(lineSideBack&&!lineAngel&&!lineSideLeft&&!lineSideRight&&(abs(myGoal_Angle) > 135||myGoal_Angle*ball_deg>0)&&abs(ball_deg)>30){
    double out_vec[2] = {sin(ball_deg * M_PI / 180.0)*80, (lineSideBack-1)*20};
    // if(comm_state != STATE_STANDALONE&&false){
    //   out_vec[0] = -out_vec[0];
    // }
	  mv_deg = atan2(out_vec[0], out_vec[1]) * (180.0 / M_PI);
		mv_power = sqrt(out_vec[0] * out_vec[0] + out_vec[1] * out_vec[1]);
    last_front_line_trace_time = now_cnt;
  }
  else if(now_cnt - last_front_line_trace_time < 500 && abs(ball_deg) > 30 && left_goal_angle < 0 && right_goal_angle > 0 && abs(mv_deg) > 90 && !lineAngel && !lineSideLeft && !lineSideRight)
  {
	  mv_deg = atan2(sin(ball_deg * M_PI / 180.0), 0) * (180.0 / M_PI)*1.1;
    mv_power = 90;
  }

  // ゴール左
  if(left_goal_angle>0&&ball_deg>0&&ball_deg<left_goal_angle&&((lineAngel&&LineAngle>135&&LineDepth)||lineSideRight||lineSideBack)&&myGoal_Radius<120&&myGoal_Radius!=0){
    GYRO_AngleOffset = 0;
    if(lineAngel){
      mv_deg = LineAngle-67.5;
    }
    else{
      mv_deg = 30;
    }
    mv_power = 80;
    // if(comm_state != STATE_STANDALONE&&false){
    //   mv_power = -80;
    // }
  }
  // ゴール右
  else if(right_goal_angle<0&&ball_deg<0&&ball_deg>right_goal_angle&&((lineAngel&&LineAngle<-135&&LineDepth)||lineSideLeft||lineSideBack)&&myGoal_Radius<120&&myGoal_Radius!=0){
    GYRO_AngleOffset = 0;
    if(lineAngel){
      mv_deg = LineAngle+67.5;
    }
    else{
      mv_deg = -30;
    }
    mv_power = 80;
    // if(comm_state != STATE_STANDALONE&&false){
    //   mv_power = -80;
    // }
  }
//
//
//
//  // -------------------------------------- //
//  // Section: 押し込み処理
//  // -------------------------------------- //
//  //
//  // ボールを敵ゴールに押し込むための状態遷移管理
//  // 待機 → 接近 → 脱出 のサイクル
//  //
//
//  // --- 押し込み処理：ボールを敵ゴールに押し込むための状態管理 ---
  applyLineOverCenterCorrection();

  switch (push_state)
  {
  case PushBallToGoalState::PREPARE_WAITING:
  case PushBallToGoalState::WAITING:
  case PushBallToGoalState::WAITING_GRACE:
    if (abs(enemyGoal_Angle - ball_deg) < 60 && !holding_ball&&abs(ball_deg)<60&&abs(enemyGoal_Angle)<60&&enemyGoal_Radius<105&&enemyGoal_Angle_Range>30)
    {
      if (lineAngel)
      {
          if(50<sqrt(LINE_X*LineX+LineY*LineY)&&abs(enemyGoal_Angle)<30&&abs(LineAngle)<30&&!(lineSideLeft>1||lineSideRight>1) && (now_cnt - push_wait_start_time) > 200){
            mv_power = 0;
          }

        if (abs(LineAngle - enemyGoal_Angle) < 60 && abs(enemyGoal_Angle - ball_deg) < 60)
        {
          if (push_state == PushBallToGoalState::PREPARE_WAITING)
          {
            push_wait_start_time = now_cnt;
            push_state = PushBallToGoalState::WAITING;
          }
          else
          {
            push_state = PushBallToGoalState::WAITING;
            if ((now_cnt - push_wait_start_time) > 800)
            {
              push_state = PushBallToGoalState::APPROACHING;
              push_approach_start_time = now_cnt;
              LineAngle_before_Approaching = LineAngle;
            }
          }
        }
        else
        {
          push_state = PushBallToGoalState::PREPARE_WAITING;
        }

        if(abs(ball_deg)>45 && enemyGoal_Radius < 105){
          mv_deg = ball_deg/abs(ball_deg)*100;
          mv_power = fmax(abs(ball_deg)*4,80.0);
        }

      }
      else
      {
        if (push_state == PushBallToGoalState::WAITING_GRACE || abs(ball_deg)>60)
        {
          if ((now_cnt - push_line_escaped_time_when_waiting) > 300)
          {
            push_state = PushBallToGoalState::PREPARE_WAITING;
          }
        }
        else if (push_state == PushBallToGoalState::WAITING)
        {
          push_state = PushBallToGoalState::WAITING_GRACE;
          push_line_escaped_time_when_waiting = now_cnt;
        }
      }
    }
    else
    {
      push_state = PushBallToGoalState::PREPARE_WAITING;
    }
    break;
  case PushBallToGoalState::APPROACHING:
    mv_deg = ball_deg;
    GYRO_AngleOffset = 0;
    mv_power = 80;
    dribbler_power = 0;
    if (LineY < -96 || (now_cnt - push_approach_start_time) > 400)
    {
      push_state = PushBallToGoalState::ESCAPING;
      push_escape_start_time = now_cnt;
    }
    break;
  case PushBallToGoalState::ESCAPING:
    mv_deg = LineAngle_before_Approaching + 180;
    GYRO_AngleOffset = 0;
    mv_power = 80;
    if (!lineAngel && (now_cnt - push_escape_start_time) > 400)
    {
      push_state = PushBallToGoalState::PREPARE_WAITING;
    }
    break;
  }



  if (is_role_changed)
  {
    if (lineAngel)
    {
      mv_power = 80;
      if (myGoal_Width > 50)
      {
        mv_deg = myGoal_Angle + 180;
      }
      else
      {
        mv_deg = LineAngle + 180;
      }
    }
    else
    {
      is_role_changed = false;
    }
  }

  applyLineLossReturnCorrection();
}
