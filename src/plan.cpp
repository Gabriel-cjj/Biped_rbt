
#include"plan.h"
#include<cmath>
#include<iostream>
#include"kinematics.h"
using namespace std;

//行走参数静态变量
static double foot_position_start_point[6] =
{
	-kBodyLong, 0, -kBodyWidth / 2,
	-kBodyLong, 0, kBodyWidth / 2
};
static double body_positon_start_point[16] = 
{ 
	1,0,0,0,
	0,1,0,kBodyHigh,
	0,0,1,0,
	0,0,0,1 };
double end_pointing[6] = 
{ 
	1,0,0,
	1,0,0
};
double end_position_on_foot[2] = { 85,85 };

//输出参数，模型曲线测试使用
double file_current_leg[6] = { 0 };
double file_current_body[16] = { 0 };
double time_test = 0;


/////////////////////////////////////梯形曲线///////////////////////////////////////////////
//生成梯形曲线0->1
//输入：时间，每毫秒计数一次
//输出：当前时刻s的值
auto TCurve::getTCurve(int count)->double
{
	//double ta = p.ta_;
	//double a = p.a_;
	//double v = p.v_;
	//double T_c = p.Tc_;
	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //三角形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //梯形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	//std::cout << s << std::endl;
	return s;
}

//计算梯形曲线的参数，由成员函数初始化，对应输入参数由构造函数初始化
auto TCurve::getCurveParam()->void
{
	if (v_ * v_ / a_ <= 1)
	{
		this->Tc_ = (a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		//安速度计算，此时给定的加速度不起作用
		this->Tc_ = 2.0 / v_;
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}

/////////////////////////////////////椭圆轨迹////////////////////////////////////////////////

//生成椭圆轨迹，在Tc时间内  x方向0->a;y方向0->b->0;z方向0->c。对应输入参数由构造函数初始化。
auto EllipseTrajectory::getEllipseTrajectory(int count)->void
{
	x_ = a_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	y_ = b_ * sin(PI - PI * s_.getTCurve(count));
	z_ = c_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	//std::cout << y << std::endl;
}

/////////////////////////////////////身体和末端的规划////////////////////////////////////////////////

//末端轨迹
//双足机器人足端在笛卡尔空间下的坐标规划
//当前脚的位置 = 上一步脚的位置 + 脚位置增量
//每当计算完一次梯形曲线，脚的位置跟新一次
//注意：#脚的位置在初始时刻要初始化一下，否则一开始只有脚1动，2脚的初始位置时0
//#注意：目前只适用于平地行走
auto planBipedRobotLeg(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	//if (count == 0)//初始化脚的位置
	//{
	//	for (int i = 0; i < 6; i++)
	//	{
	//		current_leg[i] = foot_position_start_point[i];
	//	}
	//}
	//Ellipse->getEllipseTrajectory(count);

	//if (e_1 % 2 == 0)  //1迈腿2停
	//{
	//	if (e_1 == 0)   //加速段
	//	{
	//		current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
	//		current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
	//		current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
	//		current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
	//		current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();
	//	}
	//}
	//else if (e_1 % 2 == 1)  //2迈腿1停
	//{
	//	if (e_1 == 2 * n - 1)//减速段
	//	{
	//		current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
	//		current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
	//		current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
	//		current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
	//		current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
	//	}
	//}

	//if (count + 1 == floor(Ellipse->getTcurve().getTc() * 1000))
	//{
	//	for (int i = 0; i < 6; i++)
	//	{
	//		foot_position_start_point[i] = current_leg[i];
	//	}
	//}

	if (count == 0)//初始化脚的位置，否则24脚初始位置为0
	{
		for (int i = 0; i < 6; i++)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);

	if (e_1 % 2 == 0)  //偶数13迈腿，24停
	{
		if (e_1 == 0)   //加速段
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;

		}
		else
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();

		}
	}
	else if (e_1 % 2 == 1)  //奇数24迈腿13停
	{
		if (e_1 == (2 * n - 1))//减速段
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;

		}
		else
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();

		}
	}

	if (count + 1 == floor(Ellipse->getTcurve().getTc() * 1000))
	{
		for (int i = 0; i < 6; i++)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//身体轨迹
//本函数用于规划双足机器人在对角步态下身体的位置轨迹，不加旋转（姿态变换）
//当前身体的位置 = 上一步身体的位置 + 身体位置增量
//每当结束一次命令是，身体的位置跟新一次
//#注意：目前只适用于平地行走
auto planBipedRobotBodyforWalk(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	if (count == 0) //有用，不能删，否则算不出角度
	{
		for (int i = 0; i < 16; i++)
		{
			current_body[i] = body_positon_start_point[i];
		}
	}

	if (e_1 == 0)   //加速段
	{
		current_body[3] = body_positon_start_point[3] + Ellipse->get_a() * count * count / (4.0 * per_step_count * per_step_count);
		current_body[7] = body_positon_start_point[7];
		current_body[11] = body_positon_start_point[11] + Ellipse->get_c() * count * count / (4.0 * per_step_count * per_step_count);
	}
	else if (e_1 == n - 1)//减速段
	{
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = body_positon_start_point[3] + 0 - Ellipse->get_a() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_a() * n - Ellipse->get_a() / 2.0;//n * a
		current_body[7] = body_positon_start_point[7];
		current_body[11] = body_positon_start_point[11] + 0 - Ellipse->get_c() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_c() * n - Ellipse->get_c() / 2.0;
	}
	else //匀速段
	{
		current_body[3] = body_positon_start_point[3] + Ellipse->get_a() / 4.0 + Ellipse->get_a() * (count - per_step_count) / per_step_count / 2;//速度为100mm/s  每秒计数per_step_count
		current_body[7] = body_positon_start_point[7];
		current_body[11] = body_positon_start_point[11] + Ellipse->get_c() / 4.0 + Ellipse->get_c() * (count - per_step_count) / per_step_count / 2;
	}

	if (count + 1 >= 2 * n * per_step_count)
	{
		for (int i = 0; i < 16; i++)
		{
			body_positon_start_point[i] = current_body[i];
		}
	}
}

auto planBipedRobotBodyforWalkStep(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	if (count == 0) 
	{
		for (int i = 0; i < 16; i++)
		{
			current_body[i] = body_positon_start_point[i];
		}
	}

	if (e_1 == 0)   //加速段
	{
		current_body[3] = body_positon_start_point[3];
		current_body[7] = body_positon_start_point[7];
		current_body[11] = body_positon_start_point[11];
	}
	else if (e_1 == n - 1)//减速段
	{
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = body_positon_start_point[3];
		current_body[7] = body_positon_start_point[7];
		current_body[11] = body_positon_start_point[11];
	}
	else //匀速段
	{
		current_body[3] = body_positon_start_point[3];
		current_body[7] = body_positon_start_point[7];
		current_body[11] = body_positon_start_point[11];
	}

	if (count + 1 >= 2 * n * per_step_count)
	{
		for (int i = 0; i < 16; i++)
		{
			body_positon_start_point[i] = current_body[i];
		}
	}
}

/////////////////////////////////////步态规划////////////////////////////////////////////////

//机器人行走步态，包括原地踏步、前进、后退、左移、右移。
//其中步长步高和步数可由用户输入。走一步的时间（或行走快慢）可由用户输入梯形曲线的速度和加速度确定
//#注意：行走最大速度和加速度还没测试
auto forwardBipedRobot(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	static double current_leg_in_ground[6] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步e1加1

	//规划腿
	planBipedRobotLeg(e_1, n, current_leg_in_ground, count % per_step_count, Ellipse);
	//规划身体
	planBipedRobotBodyforWalk(e_1, n, current_body_in_ground, count, Ellipse);

	//模型测试使用
	for (int j = 0; j < 6; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 16; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}

	//模型测试使用
	inverseCalculation(current_leg_in_ground, current_body_in_ground, end_pointing, end_position_on_foot, input);

	return 2 * n * per_step_count - count - 1;
}

