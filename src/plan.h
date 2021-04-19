#ifndef PLAN_H_
#define PLAN_H_

////���������ģ�����߲���ʹ��
//double file_current_leg[6] = { 0 };
//double file_current_body[16];
//double time_test;

///���ܣ�����0->1���������ߡ��ɸ�������ļ��ٶȺ��ٶ��ж�����Ϊ���λ���������
//   ##��������##
//  Tc:���������ܹ���Ҫ��ʱ�䣬������ļ��ٶȺ��ٶȼ���
//   v:�ٶȣ����û����룬���캯����ʼ��
//   a:���ٶȣ����û����룬���캯����ʼ��
//  ta:���ٶ������ʱ�䣬��������ٶȺͼ��ٶȼ���õ�
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

///���ܣ�������Բ�켣����Tcʱ����  x����0->a;y����0->b->0;z����0->c
//   ##��������##
//   a:x���򲽳������û����룬���캯����ʼ��
//   b:y���򲽸ߣ����û����룬���캯����ʼ��
//   c:z���򲽳������û����룬���캯����ʼ��
//   x:x������tʱ��ʱ��λ��
//   y:y������tʱ��ʱ��λ��
//   z:z������tʱ��ʱ��λ��
//   s:��������
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







/***********************������������***********************/
auto forwardBipedRobot(int n, int count, EllipseTrajectory* Ellipse, double* input)->int;
#endif




