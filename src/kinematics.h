#ifndef KINEMATICS_H_
#define KINEMATICS_H_

const double PI = 3.14159265358979323846;
//const double kBodyLong = 829.19; //mm  x方向
const double kBodyLong = 112; //mm  x方向
const double kBodyWidth = 405;   //mm  z方向
const double kBodyHigh = 1090.67763961;     //mm  y方向
//const double kBodyHigh = 1131.9101101572;    //mm  y方向
//const double kBodyHigh = L2+L3-0.01;    //mm  y方向
double* s_pm_dot_pm(const double* pm1, const double* pm2, double* pm_out);
double* s_inv_pm_dot_pm(const double* inv_pm, const double* pm, double* pm_out);
auto inverseCalculation(double* leg_in_ground, double* body_in_ground, double* end_pointing, double* end_position_on_foot, double* input)->int;
void ikForBipedRobotforTest(double x, double y, double z, double a, double b, double c, double l, double input[6]);
#endif
