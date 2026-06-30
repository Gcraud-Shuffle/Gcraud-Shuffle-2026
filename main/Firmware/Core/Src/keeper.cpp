// keeper.cpp – キーパールーチン
#include "robot.hpp"
#include <algorithm>
using namespace std;

#define blueGoalAngle 0
#define blueGoalWidth 1
#define yelGoalAngle 2
#define yelGoalWidth 3

bool got_push = false;
double pre_line_vec[2] = {0,0.1};
double before_push = 0;
bool restart = true;
uint32_t last_ball_moved_time;
uint32_t start_approach_time;
double freeze_ball_deg = 0;
double low_passed_ball_deg = 0;
uint32_t pw_start_time;
bool jyro_ok_flag = true;
uint32_t last_IR_exist_time;

typedef enum
{
	NOT_NEED2APPROACH,
	BALL_IS_FREEZE,
	APPROACHING,
	MOVING_AS_FORWARD
} approach_breakdown_state;

approach_breakdown_state now_keeper_state = NOT_NEED2APPROACH;

void keeper()
{
	uint32_t now_cnt = HAL_GetTick();
	static uint32_t pre_cnt = now_cnt;
	low_passed_ball_deg += (ball_deg - low_passed_ball_deg)/10;
	// --------------------------------------- //
	// Section: A New Role
	// --------------------------------------- //

	if (is_role_changed)
	{
		restart = true;
		is_role_changed = false;
		now_keeper_state = NOT_NEED2APPROACH;
	}

	if (Ball_Closeness > 30)
	{
		last_IR_exist_time = now_cnt;
	}

	if (((int)(now_cnt - last_IR_exist_time) % 2000) > 1500 && 8000 < (now_cnt - last_IR_exist_time) && (now_cnt - last_IR_exist_time) < 14000)
	{
		use_buzzer_in_algo = true;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
	}

	// --------------------------------------- //
	// Section: Revenge	of the keeper
	// --------------------------------------- //

	if(now_keeper_state == APPROACHING){
		if(!lineAngel && !lineSideBack && !lineSideLeft && !lineSideRight && (start_approach_time - now_cnt > 300)){
			is_role_changed = true;
			now_keeper_state = MOVING_AS_FORWARD;
			forward();
			force_role_forward();
			return;
		}
		GYRO_AngleOffset = (ball_deg - static_cast<int>(e.z))*1.5;
		mv_power = 50;
		mv_deg = ball_deg*1.5;
		return;
	}

	// --------------------------------------- //
	// Section: ロボ達(?)の帰還
	// --------------------------------------- //

	if (abs(ball_deg) > 90 || ((0 > ball_deg * myGoal_Angle) && ((!lineSideLeft && (ball_deg < -22.5)) || (!lineSideRight && (ball_deg > 22.5)))))
	{
		dribbler_power = 0;
	}

	if (40 < enemyGoal_Width)
	{
		restart = true;
		got_push = false;
	}

	// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, restart ? GPIO_PIN_SET : GPIO_PIN_RESET);
	if (restart)
	{
		now_keeper_state = NOT_NEED2APPROACH;
		if (lineAngel && (myGoal_Width != 0) && (90 < abs(myGoal_Angle)) && rotateMotor)
		{
			restart = false;
		}
		else
		{
			if (jyro_ok_flag && abs(GYRO_E) < 5.0 && (myGoal_Width != 0) && (90 < abs(myGoal_Angle)))
			{
				mv_deg = myGoal_Angle;
				mv_power = 60;
			}
			else if (jyro_ok_flag && abs(GYRO_E) < 10.0)
			{
				mv_deg = 180;
				mv_power = 60;
			}
			else if (jyro_ok_flag && (lineAngel || lineSideBack || lineSideRight || lineSideLeft))
			{
				double line_vec[2] = {LineX != 0 ? LineX : (127.0 * ((bool)lineSideRight) - 127.0 * ((bool)lineSideLeft)),
									  LineY != 0 ? LineY : ((((bool)lineSideBack) ? -127.0 : ((bool)lineSideRight) * ((bool)lineSideLeft) * 0.1))};

				// --- out ---
				double out_vec[2] = {-line_vec[0], -line_vec[1]};

				mv_deg = atan2(out_vec[0], out_vec[1]) * (180.0 / M_PI);
				mv_power = 100;
			}
			else
			{
				if (rotateMotor)
				{
					if (abs(GYRO_E) > 20.0)
					{
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
						use_buzzer_in_algo = true;
						jyro_ok_flag = false;
						pw_start_time = HAL_GetTick();
					}
					else if (now_cnt - pw_start_time > 300)
					{
						jyro_ok_flag = true;
					}
				}
				mv_deg = 0;
				mv_power = 0;
			}
			last_ball_moved_time = now_cnt;
			return;
		}
	}

	// --------------------------------------- //
	// Section: キーパーの移動ベクトルの算出
	// --------------------------------------- //
	{

		/*
				input: Line(x=LineX, y=LineY) IR(r=ball_dis, theta=ball_deg) Goal(theta=myGoal_Angle)
				output: mv_deg, mv_power;

				LineX       			ロボットから見て左方向に負、右方向に正
				LineY 					ロボットから見て後ろ方向に負、前方向に正

				myGoal_Angle			前が0、右回りに正、左回りに負
				ball_deg  				前が0、右回りに正、左回りに負

				mv_deg    				前が0、右回りに正、左回りに負


						* * * キーパーの移動ベクトルの算出方法 * * *
			line_vec(x,y)	 			ライン中心へ戻るためのベクトル（位置補正）
			trace_norvec(x,y) 			line_vecに直交する単位ベクトル（ラインの接線方向）
			IR_vec(x,y) 				ボール方向の単位ベクトル
			Goal_vec(x,y) 				ゴール方向の単位ベクトル

				--- pattern 0 ---
			trace_vec(x,y) 				IR_vecをtrace_norvec方向に射影した成分（ラインに沿ったボール方向成分）

				--- pattern 1 ---
			trace_vec(x,y)				IR_vecとGoal_vecを結ぶ直線上で、trace_norvecと平行（外積=0）となる点を補間で求めたベクトル
			out_vec(x,y) 				line_vecとtrace_vecの線形結合（現在はline_vecのみ使用）

				--- pattern 2 ---
			trace_vec(x,y)				pattern1のtrace_dotをPID制御気によってゼロにしようとする制御が入った成分
		*/

		if (lineAngel || lineSideBack || lineSideRight || lineSideLeft)
		{
			double trace_gain = 120.0;
			double Line_gain = 0.6;
			double mv_gap = 10.0;
			double trace_vec[2] = {0, 0}; // ライン追従調整用

			double trace_ignore_goal_abs_thr[3] = {140, 130, 120};



			//				double test_deg = 45.0; // デバッグ用

			double line_vec[2] = {lineAngel? LineX : (127.0 * ((bool)lineSideRight) - 127.0 * ((bool)lineSideLeft)),
								  lineAngel? LineY : ((((bool)lineSideBack) ? -127.0 : ((bool)lineSideRight) * ((bool)lineSideLeft) * 0.1))};

			if (abs(line_vec[0]) < 0.1 && abs(line_vec[1]) < 0.1)
			{
				line_vec[0] = pre_line_vec[0];
				line_vec[1] = pre_line_vec[1];
			}

			double DepthOfLine = sqrt(line_vec[0] * line_vec[0] + line_vec[1] * line_vec[1]);

			// double line_vec[2] = {LineX, LineY};
			double trace_norvec[2] = {-line_vec[1] / DepthOfLine, line_vec[0] / DepthOfLine};
			double IR_vec[2] = {sin(ball_deg * M_PI / 180.0) * 1.6, cos(ball_deg * M_PI / 180.0)};
			//				double IR_vec[2] = {sin(test_deg * M_PI
			/// 180.0), cos(test_deg * M_PI / 180.0)};

			double Goal_vec[2] = {sin(myGoal_Angle * M_PI / 180.0), cos(myGoal_Angle * M_PI / 180.0)};

			double IR_trace_dot = IR_vec[0] * trace_norvec[0] + IR_vec[1] * trace_norvec[1];

			trace_vec[0] = trace_norvec[0] * IR_trace_dot;
			trace_vec[1] = trace_norvec[1] * IR_trace_dot;

			if (left_goal_angle > 0 || right_goal_angle < 0)
			{
				use_buzzer_in_algo = true;
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
			}

			// 対岸時のゴール真横脱出
			if ((160 < abs(ball_deg)) && (170 > abs(ball_deg)) && (0 < ball_deg * myGoal_Angle) && (trace_vec[1] < 0.0) && (abs(myGoal_Angle) < trace_ignore_goal_abs_thr[0]))
			{
				trace_gain = -trace_gain;
				use_buzzer_in_algo = true;
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
			}
			// アウト回避(後ろ方向にボールがいる時/横にいきすぎないように)
			else if ((trace_vec[1] < 0.0))
			{
				if (abs(myGoal_Angle) < trace_ignore_goal_abs_thr[2])
				{
					trace_vec[0] = 0;
					trace_vec[1] = 1.0;
				}

				else if (abs(myGoal_Angle) < trace_ignore_goal_abs_thr[1])
				{
					trace_vec[0] = 0;
					trace_vec[1] = 1.0 * ((trace_ignore_goal_abs_thr[1] - abs(myGoal_Angle)) / (trace_ignore_goal_abs_thr[1] - trace_ignore_goal_abs_thr[2]));
				}
				else if (abs(myGoal_Angle) < trace_ignore_goal_abs_thr[0])
				{
					trace_gain = 0.0;
				}
			}

			// ゴール真横脱出
			if ((((line_vec[1] < -48.0)) || ((bool)lineSideRight && (bool)lineSideLeft)) && (abs(myGoal_Angle) < trace_ignore_goal_abs_thr[1]))
			{
				Line_gain = 0.0;
				trace_vec[0] = 0.0;
				trace_vec[1] = 1.0;
				trace_gain = 100.0;
			}

			// --- out ---
			double out_vec[2] = {line_vec[0] * Line_gain + trace_vec[0] * trace_gain, line_vec[1] * Line_gain + trace_vec[1] * trace_gain};
			mv_deg = atan2(out_vec[0], out_vec[1]) * (180.0 / M_PI);
			mv_power = sqrt(out_vec[0] * out_vec[0] + out_vec[1] * out_vec[1]);
			pre_line_vec[0] = line_vec[0];
			pre_line_vec[1] = line_vec[1];

			// ゴール真横ライン脱出
			if(((bool)lineSideLeft||LineAngle>0.0)&&(0<left_goal_angle&&left_goal_angle<120)||((bool)lineSideRight||LineAngle<0.0)&&(0>right_goal_angle&&right_goal_angle>-120)||myGoal_Radius>110){
				mv_deg = -atan2(line_vec[0], line_vec[1]) * (180.0 / M_PI);
				mv_power = 100;
			}

			// --------------------------------------- //
			// Section: STANDALONE時のボールへのアプローチ(フリーズ検知)
			// --------------------------------------- //
			// if(Ball_Closeness > 40 && abs(ball_deg) < 90 && abs(freeze_ball_deg - low_passed_ball_deg) < 10 && !holding_ball && !got_push){
			// 	if(int(now_cnt - last_ball_moved_time) > (comm_state == STATE_STANDALONE) ? 3800 : 5800){
			// 		now_keeper_state = APPROACHING;
			// 		start_approach_time = now_cnt;
			// 	}
			// 	else if(int(now_cnt - last_ball_moved_time) % 1000 > 800){
			// 		use_buzzer_in_algo = true;
			// 		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
			// 	}
			// }
			// else{
			// 	last_ball_moved_time = now_cnt;
			// 	freeze_ball_deg = ball_deg;
			// }

			//
			// mv_power = mv_power - min(max((double)mv_power, -mv_gap), mv_gap);
			if (got_push)
			{
				mv_deg = 0;
				mv_power = 100;
			}
		}
		else
		{
			mv_deg = myGoal_Angle;
			mv_power = 80;
		}
	}

	if (got_push)
	{
		last_ball_moved_time = now_cnt;
		mv_deg = 0;
		mv_power = 100;
	}
}
