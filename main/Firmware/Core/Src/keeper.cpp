// keeper.cpp – キーパールーチン
#include "robot.hpp"
#include <algorithm>
using namespace std;

#define blueGoalAngle 0
#define blueGoalWidth 1
#define yelGoalAngle 2
#define yelGoalWidth 3

bool got_push = false;
double pre_line_vec[2];
double before_push = 0;
bool restart = true;
uint32_t last_ball_moved_time;
uint32_t start_approach_time;
int last_ball_deg;
double use_IR_deg;

typedef enum
{
	NOT_NEED2APPROACH,
	BALL_IS_FREEZE,
	APPROACHING
} approach_breakdown_state;

approach_breakdown_state now_keeper_state = NOT_NEED2APPROACH;

void keeper()
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
			if (abs(GYRO_E) < 10.0)
			{
				mv_deg = 180;
				mv_power = 60;
			}
			else if ((myGoal_Width != 0) && (abs(myGoal_Angle) < 135))
			{
				mv_deg = myGoal_Angle;
				mv_power = 120;
			}
			else if (lineAngel || lineSideBack || lineSideRight || lineSideLeft)
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
				mv_deg = 0;
				mv_power = 0;
			}
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
		*/

		if (lineAngel || ((lineSideBack || lineSideRight || lineSideLeft) && (!got_push)))
		{
			double trace_gain = 120.0;
			double Line_gain = 0.6;
			double mv_gap = 10.0;

			double trace_ignore_goal_abs_thr[3] = {145, 135, 120};

			//				double test_deg = 45.0; // デバッグ用

			double line_vec[2] = {LineX != 0 ? LineX : (127.0 * ((bool)lineSideRight) - 127.0 * ((bool)lineSideLeft)),
								  LineY != 0 ? LineY : ((((bool)lineSideBack) ? -127.0 : ((bool)lineSideRight) * ((bool)lineSideLeft) * 0.1))};
			if (abs(line_vec[0]) < 1e-9 && abs(line_vec[1]) < 1e-9)
			{
				line_vec[0] = pre_line_vec[0];
				line_vec[1] = pre_line_vec[1];
			}

			double DepthOfLine = sqrt(line_vec[0] * line_vec[0] + line_vec[1] * line_vec[1]);

			got_push = (+64.0 < line_vec[1]);
			if (got_push)
			{
				before_push = atan2(line_vec[0], line_vec[1]) * (180.0 / M_PI);
			}

			// double line_vec[2] = {LineX, LineY};
			double trace_norvec[2] = {-line_vec[1] / DepthOfLine, line_vec[0] / DepthOfLine};
			double IR_vec[2] = {sin(ball_deg * M_PI / 180.0), cos(ball_deg * M_PI / 180.0)};
			//				double IR_vec[2] = {sin(test_deg * M_PI
			/// 180.0), cos(test_deg * M_PI / 180.0)};

			double Goal_vec[2] = {sin(myGoal_Angle * M_PI / 180.0), cos(myGoal_Angle * M_PI / 180.0)};

			//				 double trace_vec[2] = {0, 0}; //
			// ライン追従調整用

			// pattern 0 :
			// IR_vecをtrace_norvec方向へ射影（内積）してライン接線方向成分を抽出
			double IR_trace_dot = (IR_vec[0] * trace_norvec[0] + IR_vec[1] * trace_norvec[1]);
			double trace_vec[2] = {trace_norvec[0] * IR_trace_dot, trace_norvec[1] * IR_trace_dot};

			// pattern 1 :
			// IR_vecとGoal_vec間の補間で、trace_norvecと平行（外積=0）となるベクトルを求める
			if (Ball_Closeness < 50)
			{
				double cross1 = IR_vec[0] * trace_norvec[1] - IR_vec[1] * trace_norvec[0];
				double IR2GOAL[2] = {Goal_vec[0] - IR_vec[0], Goal_vec[1] - IR_vec[1]};
				double cross2 = IR2GOAL[0] * trace_norvec[1] - IR2GOAL[1] * trace_norvec[0];
				// trace_vec[0] = IR_vec[0] - cross1 / cross2 * IR2GOAL[0];
				// trace_vec[1] = IR_vec[1] - cross1 / cross2 * IR2GOAL[1];
			}

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

			// ゴール横脱出
			if ((((line_vec[1] < -48.0)) || ((bool)lineSideRight && (bool)lineSideLeft)) && (abs(myGoal_Angle) < trace_ignore_goal_abs_thr[1]))
			{
				line_vec[0] = 0.0;
				line_vec[1] = 0.0;
				trace_vec[0] = 0;
				trace_vec[1] = 1.0;
				trace_gain = 100.0;
			}

			// --- out ---
			double out_vec[2] = {line_vec[0] * Line_gain + trace_vec[0] * trace_gain, line_vec[1] * Line_gain + trace_vec[1] * trace_gain};

			mv_deg = atan2(out_vec[0], out_vec[1]) * (180.0 / M_PI);
			mv_power = sqrt(out_vec[0] * out_vec[0] + out_vec[1] * out_vec[1]);
			pre_line_vec[0] = line_vec[0];
			pre_line_vec[1] = line_vec[1];
			// mv_power = mv_power - min(max((double)mv_power, -mv_gap), mv_gap);
		}
		else if (got_push)
		{
			mv_deg = before_push;
			mv_power = 100;
		}
		else
		{
			mv_deg = myGoal_Angle;
			mv_power = 80;
		}
	}

	// --------------------------------------------- //
	// Section: 故障判定回避
	// -------------------------------------------- //
	// {
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
					-> 検知してから1.5~2秒ぐらい後に0.5~1秒間ぐらい直線運動でアプローチを試み2秒以内に戻る

				- 相手陣側へボールを移動
					-> ボールに与えられる慣性次第(何秒間押し続けるかの調整)
	 */
	/*
			uint32_t now = HAL_GetTick();
			use_IR_deg += (ball_deg - use_IR_deg) / 10;

			if (
				((current_state == STATE_STANDALONE) && !is_role_changed && !restart) && // ロボットの状態的なsomething
				(abs(ball_deg) < 70.0) && (ball_dis < 40.0) &&							 // ボールの大体の位置
				(abs(use_IR_deg - last_ball_deg) < 10) &&								 // 動いてるかどうか
				(now_keeper_state != APPROACHING) && lineAngel)							 // アプローチ中は例外
			{
				if ((now - last_ball_moved_time) < 2222)
				{
					now_keeper_state = BALL_IS_FREEZE;
				}
				else
				{
					now_keeper_state = APPROACHING;
					start_approach_time = now;
				}
			}
			else if (now_keeper_state != APPROACHING)
			{
				now_keeper_state = NOT_NEED2APPROACH;
			}

			if (now_keeper_state == NOT_NEED2APPROACH)
			{
				last_ball_moved_time = now;
				last_ball_deg = ball_deg;
				use_IR_deg = ball_deg;
			}
			else if (now_keeper_state == APPROACHING)
			{
				if ((now - start_approach_time) < 200 && (50 < myGoal_Width))
				{
					mv_deg = ball_deg;
					mv_power = 100;
					if (holding_ball && !kicking_active)
					{
						kicking_active = true;
						kicking_start_time = cnt;
					}
				}
				else
				{
					now_keeper_state = NOT_NEED2APPROACH;
					last_ball_moved_time = now;
					last_ball_deg = ball_deg;
					use_IR_deg = ball_deg;
				}
			}
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 300 < (now - last_ball_moved_time) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		}*/
}
