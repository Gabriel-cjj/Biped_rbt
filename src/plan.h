#ifndef PLAN_H_
#define PLAN_H_

////输出参数，模型曲线测试使用
//double file_current_leg[6] = { 0 };
//double file_current_body[16];
//double time_test;

///功能：生成0->1的梯形曲线。可根据输入的加速度和速度判断曲线为梯形还是三角形
//   ##参数定义##
//  Tc:生成曲线总共需要的时间，由输入的加速度和速度计算
//   v:速度，由用户输入，构造函数初始化
//   a:加速度，由用户输入，构造函数初始化
//  ta:加速段所需的时间，由输入的速度和加速度计算得到
class TCurve
{
private:
	double Tc_;
	double v_;
	double a_;
	double ta_;

public:
	auto getTCurve(int count)->double;
	auto getCurveParam()->void;
	auto getTc()->double { return Tc_; };
	TCurve(double a, double v) { a_ = a; v_ = v; }
	~TCurve() {}
};

///功能：生成椭圆轨迹。在Tc时间内  x方向0->a;y方向0->b->0;z方向0->c
//   ##参数定义##
//   a:x方向步长，由用户输入，构造函数初始化
//   b:y方向步高，由用户输入，构造函数初始化
//   c:z方向步长，由用户输入，构造函数初始化
//   x:x方向在t时刻时的位置
//   y:y方向在t时刻时的位置
//   z:z方向在t时刻时的位置
//   s:梯形曲线
class EllipseTrajectory
{
private:
	double x_;
	double y_;
	double z_;
	double a_;
	double b_;
	double c_;
	TCurve s_;

public:
	auto getEllipseTrajectory(int count)->void;
	auto get_x()->double { return x_; };
	auto get_y()->double { return y_; };
	auto get_z()->double { return z_; };
	auto get_a()->double { return a_; };
	auto get_b()->double { return b_; };
	auto get_c()->double { return c_; };
	auto getTcurve()->TCurve { return s_; };
	EllipseTrajectory(double a, double b, double c, TCurve& s) :a_(a), b_(b), c_(c), s_(s), x_(0), y_(0), z_(0) {}
	~EllipseTrajectory() {}
};







/***********************函数变量声明***********************/
auto forwardBipedRobot(int n, int count, EllipseTrajectory* Ellipse, double* input)->int;
#endif




