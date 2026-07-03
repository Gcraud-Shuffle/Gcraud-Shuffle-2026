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
double use_IR_deg;
bool jyro_ok_flag = true;
uint32_t last_IR_exist_time;
uint32_t pw_start_time;
float diff;

typedef enum
{
	NOT_NEED2APPROACH,
	BALL_IS_FREEZE,
	APPROACHING
} approach_breakdown_state;

approach_breakdown_state now_keeper_state = NOT_NEED2APPROACH;

void keeper()
{
	// --------------------------------------- //
	// Section: Default Strategies
	// --------------------------------------- //
	uint32_t now_cnt = HAL_GetTick();
	if(holding_ball){
		request_kick();
	}

	// --------------------------------------- //
	// Section: Revenge	of the keeper
	// --------------------------------------- //
	{
		if(now_keeper_state == APPROACHING){
			if((now_cnt - start_approach_time) > 100){
				is_role_changed = true;
				now_keeper_state = NOT_NEED2APPROACH;
				forward();
				force_role_forward();
				return;
			}
			GYRO_AngleOffset = (ball_deg - static_cast<int>(e.z));
			mv_power = 50;
			mv_deg = ball_deg*1.5;
			return;
		}
	}

	// --------------------------------------- //
	// Section: キーパーの起動直後の帰還動作
	// --------------------------------------- //
	{
		if (is_role_changed)
		{
			restart = true;
			is_role_changed = false;
			now_keeper_state = NOT_NEED2APPROACH;
		}

		if (40 < enemyGoal_Width)
		{
			restart = true;
			got_push = false;
		}

		if (restart)
		{
			now_keeper_state = NOT_NEED2APPROACH;
			if (lineAngel && (myGoal_Width != 0) && (90 < abs(myGoal_Angle)))
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

			trace_vec(x,y) 				IR_vecをtrace_norvec方向に射影した成分（ラインに沿ったボール方向成分）

		ultrasonic memo
		1	robot 	0
			  2
		*/

		if (lineAngel || ((lineSideBack || lineSideRight || lineSideLeft) && (!got_push)))
		{
			double trace_gain = 110.0;
			double Line_gain = 0.7;
			double mv_gap = 10.0;

			double trace_ignore_goal_abs_thr[3] = {155, 145, 130};
			// 0:traceを完全に無効化する境界 (停止)
			// 1:前進成分への補間開始・ゴール横脱出条件 (中途前進)
			// 2:完全に前進のみへ置き換える境界 (完全前進)

			//				double test_deg = 45.0; // デバッグ用

			double line_vec[2] = {lineAngel? LineX : (127.0 * ((bool)lineSideRight) - 127.0 * ((bool)lineSideLeft)),
								  lineAngel? LineY : ((((bool)lineSideBack) ? -127.0 : ((bool)lineSideRight) * ((bool)lineSideLeft) * 0.1))};
			if (abs(line_vec[0]) < 1e-9 && abs(line_vec[1]) < 1e-9)
			{
				line_vec[0] = pre_line_vec[0];
				line_vec[1] = pre_line_vec[1];
			}

			double DepthOfLine = sqrt(line_vec[0] * line_vec[0] + line_vec[1] * line_vec[1]);

			// double line_vec[2] = {LineX, LineY};
			double trace_norvec[2] = {-line_vec[1] / DepthOfLine, line_vec[0] / DepthOfLine};
			double IR_vec[2] = {sin(ball_deg * M_PI / 180.0), cos(ball_deg * M_PI / 180.0)};
			//				double IR_vec[2] = {sin(test_deg * M_PI
			/// 180.0), cos(test_deg * M_PI / 180.0)};

			double Goal_vec[2] = {sin(myGoal_Angle * M_PI / 180.0), cos(myGoal_Angle * M_PI / 180.0)};

							//  double trace_vec[2] = {0, 0}; //
			// ライン追従調整用

			// IR_vecをtrace_norvec方向へ射影（内積）してライン接線方向成分を抽出
			double IR_trace_dot = (IR_vec[0] * trace_norvec[0] + IR_vec[1] * trace_norvec[1]);
			double trace_vec[2] = {trace_norvec[0] * IR_trace_dot, trace_norvec[1] * IR_trace_dot};

			// アウト回避(後ろ方向にボールがいる時)
			if ((160 < abs(ball_deg)) && (0 < ball_deg * myGoal_Angle) && (trace_vec[1] < 0.0) && (abs(myGoal_Angle) < trace_ignore_goal_abs_thr[0]))
			{
				trace_gain = -trace_gain;
			}
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

			// --- out ---
			double out_vec[2] = {line_vec[0] * Line_gain + trace_vec[0] * trace_gain, line_vec[1] * Line_gain + trace_vec[1] * trace_gain};

			mv_deg = atan2(out_vec[0], out_vec[1]) * (180.0 / M_PI);
			mv_power = sqrt(out_vec[0] * out_vec[0] + out_vec[1] * out_vec[1]);
			pre_line_vec[0] = line_vec[0];
			pre_line_vec[1] = line_vec[1];

			if(myGoal_Width==0){
				if(MainSub_SafeUltrasonic_mm[0] < MainSub_SafeUltrasonic_mm[1])//左が広い
				{
					mv_deg = -45;
					mv_power = 80;
				}
				else{
					mv_deg = 45;
					mv_power = 80;
				}
			}
			else if(left_goal_angle*right_goal_angle > 0 && myGoal_Width != 0 && 150 >  max(abs(left_goal_angle),abs(right_goal_angle))){
				// ゴール横脱出(横棒)
				if(((lineSideLeft&&lineSideRight&&lineAngel)||((!lineSideLeft)&&(!lineSideRight)&&(!lineAngel)&&(lineSideBack)))){
					if(myGoal_Angle < 0){//左側
						mv_deg = -70;
						mv_power = 80;
					}
					else{
						mv_deg = 70;
						mv_power = 80;
					}
				}
				// ゴール横脱出(縦棒)
				else{
					if(myGoal_Angle < 0){//左側
						mv_deg = -70;
						mv_power = 80;
					}
					else{
						mv_deg = 70;
						mv_power = 80;
					}
				}
			}
		}
		else
		{
			if(myGoal_Width==0){
				if(MainSub_SafeUltrasonic_mm[0] < MainSub_SafeUltrasonic_mm[1])//左が広い
				{
					mv_deg = -45;
					mv_power = 80;
				}
				else{
					mv_deg = -45;
					mv_power = 80;
				}
			}
			else{
				mv_deg = myGoal_Angle;
				mv_power = 80;
			}
		}
	}

	// ---------------------------------------------- //
	// Section: 押し出され対応
	// ---------------------------------------------- //
	{
		diff = myGoal_Angle - LineAngle;

		while (diff > 180.0f) diff -= 360.0f;
		while (diff < -180.0f) diff += 360.0f;

		if (lineAngel&&(abs(diff)>90&&(abs(LineAngle)<135))&&LineDepth>100&&(((150 <  max(abs(left_goal_angle),abs(right_goal_angle)))||left_goal_angle*right_goal_angle < 0)&& myGoal_Width!=0)) {
			before_push = LineAngle;
			got_push = true;
		}
		else if((abs(LineAngle)<135&&LineDepth<80&&lineAngel)||(0 < left_goal_angle&&left_goal_angle < 150)||(0>right_goal_angle&&right_goal_angle>-150)||(myGoal_Radius > 110)||(myGoal_Width==0)){
			got_push = false;
		}

		if (got_push)
		{
			last_ball_moved_time = now_cnt;
			mv_deg = before_push/2;
			mv_power = 100;
		}
	}

	// --------------------------------------------- //
	// Section: 故障判定回避
	// -------------------------------------------- //
	{
		/*
			input: IR(r=ball_dis, theta=ball_deg) Goal(theta=myGoal_Angle) current_state
			output: mv_deg, mv_power;

				* * * 故障が取られるタイミングについて(前提知識) * * *
			- ルールブックからの引用
				Any robot must approach and touch the ball when it is placed on the nearest neutral spot.
				It must do this before lack of progress is called.
				When on its own side of the field,any robot must be able to move the ball from the nearest neutral spot to the opponent’s side of the field.
				If a specific robot does not act this way, referees may deem it damaged at their discretion. (See Damaged Robots.)
				This rule does not apply if the	robot is hindered from detecting or playing the ball by the opponent.

				どのロボットも、ボールが最も近いニュートラルスポットに置かれたら、ボールに近づいて触れなければいけません。
				ラックオブプログレスがコールされる前にこれを行わなければいけません。
				自陣側にいる場合、どのロボットも最も近いニュートラルスポットから相手陣側へボールを移動させることができなければいけません。
				特定のロボットがこのような行動をとった場合、審判はそのロボットをその裁量で故障とみなすことができます。
				(ロボットの故障参照)このルールは、ロボットが相手によってボールを発見したりプレイしたりすることを妨げられた場合には適用されません


			- 評価可能な方法
				- ラックオブプログレスがコールされる前
					-> 三秒間ボールが動かない(IR_degが一定偏差で保ち続ける)
					-> current_state is STATE_STANDALONEであるとき(こちらサイドがラックオブプログレス前にアプローチすることができるのはキーパーだけだから)

				- 最も近いニュートラルスポットから
					-> ball_degが特定の範囲内(abs(IR_deg) < 90は必ず成立)

				- ボールに近づいて触れなければいけません
					-> 検知してから3秒以内にフォワードになって触りに行く

				- 相手陣側へボールを移動
					-> フォワードは相手側へボールを届ける
		*/

		if(now_keeper_state == APPROACHING){
			if((now_cnt - start_approach_time) > 100){
				is_role_changed = true;
				now_keeper_state = NOT_NEED2APPROACH;
				forward();
				force_role_forward();
				return;
			}
			GYRO_AngleOffset = (ball_deg - static_cast<int>(e.z));
			mv_power = 50;
			mv_deg = ball_deg*1.5;
			return;
		}

		if(ball_dis < 50 &&
			abs(ball_deg) < 90 &&
			abs(freeze_ball_deg - low_passed_ball_deg) < 15 &&
			!holding_ball &&
			!got_push){
				if((comm_state == STATE_STANDALONE)&&(int(now_cnt - last_ball_moved_time) > 3600)){
					now_keeper_state = APPROACHING;
					start_approach_time = now_cnt;
				}
				else if(int(now_cnt - last_ball_moved_time) % 1400 > 1200){
					use_buzzer_in_algo = true;
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, period_3 * 1 / 2);
				}
			}
		else{
			last_ball_moved_time = now_cnt;
			freeze_ball_deg = low_passed_ball_deg;
		}

	}
}
