#include"kinematics.h"
#include<cmath>
#include<iostream>
#include<fstream>


extern double input_angle[10];


//身体在腿坐标系下的变换矩阵
double cs_of_leg1[16] =
{
    1, 0, 0, kBodyLong,
    0, 1, 0, 0,
    0, 0, 1, kBodyWidth / 2,
    0, 0, 0, 1
};
double cs_of_leg2[16] =
{
    1, 0, 0, kBodyLong,
    0, 1, 0, 0,
    0, 0, 1, -kBodyWidth / 2,
    0, 0, 0, 1
};


//矩阵计算
void s_inv_pm(const double* pm_in, double* pm_out)
{
    //转置
    pm_out[0] = pm_in[0];
    pm_out[1] = pm_in[4];
    pm_out[2] = pm_in[8];
    pm_out[4] = pm_in[1];
    pm_out[5] = pm_in[5];
    pm_out[6] = pm_in[9];
    pm_out[8] = pm_in[2];
    pm_out[9] = pm_in[6];
    pm_out[10] = pm_in[10];

    //位置
    pm_out[3] = -pm_out[0] * pm_in[3] - pm_out[1] * pm_in[7] - pm_out[2] * pm_in[11];
    pm_out[7] = -pm_out[4] * pm_in[3] - pm_out[5] * pm_in[7] - pm_out[6] * pm_in[11];
    pm_out[11] = -pm_out[8] * pm_in[3] - pm_out[9] * pm_in[7] - pm_out[10] * pm_in[11];

    //其他
    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;
}
double* s_pm_dot_pm(const double* pm1, const double* pm2, double* pm_out)
{
    pm_out[0] = pm1[0] * pm2[0] + pm1[1] * pm2[4] + pm1[2] * pm2[8];
    pm_out[1] = pm1[0] * pm2[1] + pm1[1] * pm2[5] + pm1[2] * pm2[9];
    pm_out[2] = pm1[0] * pm2[2] + pm1[1] * pm2[6] + pm1[2] * pm2[10];
    pm_out[3] = pm1[0] * pm2[3] + pm1[1] * pm2[7] + pm1[2] * pm2[11] + pm1[3];

    pm_out[4] = pm1[4] * pm2[0] + pm1[5] * pm2[4] + pm1[6] * pm2[8];
    pm_out[5] = pm1[4] * pm2[1] + pm1[5] * pm2[5] + pm1[6] * pm2[9];
    pm_out[6] = pm1[4] * pm2[2] + pm1[5] * pm2[6] + pm1[6] * pm2[10];
    pm_out[7] = pm1[4] * pm2[3] + pm1[5] * pm2[7] + pm1[6] * pm2[11] + pm1[7];

    pm_out[8] = pm1[8] * pm2[0] + pm1[9] * pm2[4] + pm1[10] * pm2[8];
    pm_out[9] = pm1[8] * pm2[1] + pm1[9] * pm2[5] + pm1[10] * pm2[9];
    pm_out[10] = pm1[8] * pm2[2] + pm1[9] * pm2[6] + pm1[10] * pm2[10];
    pm_out[11] = pm1[8] * pm2[3] + pm1[9] * pm2[7] + pm1[10] * pm2[11] + pm1[11];

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}
double* s_inv_pm_dot_pm(const double* inv_pm, const double* pm, double* pm_out)
{
    pm_out[0] = inv_pm[0] * pm[0] + inv_pm[4] * pm[4] + inv_pm[8] * pm[8];
    pm_out[1] = inv_pm[0] * pm[1] + inv_pm[4] * pm[5] + inv_pm[8] * pm[9];
    pm_out[2] = inv_pm[0] * pm[2] + inv_pm[4] * pm[6] + inv_pm[8] * pm[10];
    pm_out[3] = inv_pm[0] * (pm[3] - inv_pm[3]) + inv_pm[4] * (pm[7] - inv_pm[7]) + inv_pm[8] * (pm[11] - inv_pm[11]);

    pm_out[4] = inv_pm[1] * pm[0] + inv_pm[5] * pm[4] + inv_pm[9] * pm[8];
    pm_out[5] = inv_pm[1] * pm[1] + inv_pm[5] * pm[5] + inv_pm[9] * pm[9];
    pm_out[6] = inv_pm[1] * pm[2] + inv_pm[5] * pm[6] + inv_pm[9] * pm[10];
    pm_out[7] = inv_pm[1] * (pm[3] - inv_pm[3]) + inv_pm[5] * (pm[7] - inv_pm[7]) + inv_pm[9] * (pm[11] - inv_pm[11]);

    pm_out[8] = inv_pm[2] * pm[0] + inv_pm[6] * pm[4] + inv_pm[10] * pm[8];
    pm_out[9] = inv_pm[2] * pm[1] + inv_pm[6] * pm[5] + inv_pm[10] * pm[9];
    pm_out[10] = inv_pm[2] * pm[2] + inv_pm[6] * pm[6] + inv_pm[10] * pm[10];
    pm_out[11] = inv_pm[2] * (pm[3] - inv_pm[3]) + inv_pm[6] * (pm[7] - inv_pm[7]) + inv_pm[10] * (pm[11] - inv_pm[11]);

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}
double* s_pm_dot_inv_pm(const double* pm, const double* inv_pm, double* pm_out)
{
    pm_out[0] = pm[0] * inv_pm[0] + pm[1] * inv_pm[1] + pm[2] * inv_pm[2];
    pm_out[1] = pm[0] * inv_pm[4] + pm[1] * inv_pm[5] + pm[2] * inv_pm[6];
    pm_out[2] = pm[0] * inv_pm[8] + pm[1] * inv_pm[9] + pm[2] * inv_pm[10];
    pm_out[3] = -pm_out[0] * inv_pm[3] - pm_out[1] * inv_pm[7] - pm_out[2] * inv_pm[11] + pm[3];

    pm_out[4] = pm[4] * inv_pm[0] + pm[5] * inv_pm[1] + pm[6] * inv_pm[2];
    pm_out[5] = pm[4] * inv_pm[4] + pm[5] * inv_pm[5] + pm[6] * inv_pm[6];
    pm_out[6] = pm[4] * inv_pm[8] + pm[5] * inv_pm[9] + pm[6] * inv_pm[10];
    pm_out[7] = -pm_out[4] * inv_pm[3] - pm_out[5] * inv_pm[7] - pm_out[6] * inv_pm[11] + pm[7];

    pm_out[8] = pm[8] * inv_pm[0] + pm[9] * inv_pm[1] + pm[10] * inv_pm[2];
    pm_out[9] = pm[8] * inv_pm[4] + pm[9] * inv_pm[5] + pm[10] * inv_pm[6];
    pm_out[10] = pm[8] * inv_pm[8] + pm[9] * inv_pm[9] + pm[10] * inv_pm[10];
    pm_out[11] = -pm_out[8] * inv_pm[3] - pm_out[9] * inv_pm[7] - pm_out[10] * inv_pm[11] + pm[11];

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}
double* s_pm_dot_v3(const double* pm, const double* v3, double* v3_out)
{
    v3_out[0] = pm[0] * v3[0] + pm[1] * v3[1] + pm[2] * v3[2];
    v3_out[1] = pm[4] * v3[0] + pm[5] * v3[1] + pm[6] * v3[2];
    v3_out[2] = pm[8] * v3[0] + pm[9] * v3[1] + pm[10] * v3[2];

    return v3_out;
}
double* s_inv_pm_dot_v3(const double* inv_pm, const double* v3, double* v3_out)
{
    v3_out[0] = inv_pm[0] * v3[0] + inv_pm[4] * v3[1] + inv_pm[8] * v3[2];
    v3_out[1] = inv_pm[1] * v3[0] + inv_pm[5] * v3[1] + inv_pm[9] * v3[2];
    v3_out[2] = inv_pm[2] * v3[0] + inv_pm[6] * v3[1] + inv_pm[10] * v3[2];

    return v3_out;
}
double* s_pp2pp(const double* relative_pm, const double* from_pp, double* to_pp)
{
    to_pp[0] = relative_pm[0] * from_pp[0] + relative_pm[1] * from_pp[1] + relative_pm[2] * from_pp[2] + relative_pm[3];
    to_pp[1] = relative_pm[4] * from_pp[0] + relative_pm[5] * from_pp[1] + relative_pm[6] * from_pp[2] + relative_pm[7];
    to_pp[2] = relative_pm[8] * from_pp[0] + relative_pm[9] * from_pp[1] + relative_pm[10] * from_pp[2] + relative_pm[11];

    return to_pp;
}
double* s_inv_pp2pp(const double* inv_relative_pm, const double* from_pp, double* to_pp)
{
    double tem[3] = { from_pp[0] - inv_relative_pm[3] ,from_pp[1] - inv_relative_pm[7] ,from_pp[2] - inv_relative_pm[11] };

    to_pp[0] = inv_relative_pm[0] * tem[0] + inv_relative_pm[4] * tem[1] + inv_relative_pm[8] * tem[2];
    to_pp[1] = inv_relative_pm[1] * tem[0] + inv_relative_pm[5] * tem[1] + inv_relative_pm[9] * tem[2];
    to_pp[2] = inv_relative_pm[2] * tem[0] + inv_relative_pm[6] * tem[1] + inv_relative_pm[10] * tem[2];

    return to_pp;
}

//运动学反解

void ikForBipedRobot(double* ee_xyz_wrt_leg, double* end_pointing, double* end_position_on_foot, double* motion_pos) {
    //-----已知数据-----
    //L、l为长
    //v为向量
    //a为角度
    const double kLAO = 154.5;
    const double kLAD = 150;
    const double kLCD = 422.02;
    const double kLAB = 487.46;
    const double kLBC = 88.82;
    const double kLBE = 509.51;
    const double kLCE = 426.61;
    const double kLEG = 326.82;
    const double kLHI = 280;
    const double kLGH = 120;
    const double kLIJ = 85;
    const double kLEJ = 40;
    const double kLEI = sqrt(kLIJ * kLIJ + kLEJ * kLEJ);
    const double kQ30 = 2.768114255522946;
    const double kQ50 = 1.020376369330288;

    double x = ee_xyz_wrt_leg[0];
    double y = ee_xyz_wrt_leg[1];
    double z = ee_xyz_wrt_leg[2];
    double a = end_pointing[0];
    double b = end_pointing[1];
    double c = end_pointing[2];
    double l = end_position_on_foot[0];

    double v_om[3] =
    {
        x, y, z
    };
    double v_vn[3] =
    {
        a, b, c
    };

    //-----计算数据-----
    double q1 = 0, q2 = 0, q3 = 0, q4 = 0, q5 = 0;
    double i=0;//判断左右，0左1右
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    double theta1=0, theta2=0, theta3=0, theta4=0;
    double a_nop = 0, a_mpn = 0, a_lim = 0, a_eij = 0, a_eik = 0, a_cbe = 0, a_abe = 0, a_abc = 0, a_bac = 0, a_cad = 0, a_bad = 0, a_bae = 0, a_eaf = 0,
        a_daf = 0, a_adc = 0, a_aec = 0, a_aeb = 0, a_aef = 0, a_bec = 0, a_gei = 0, a_gei_b = 0, a_gei_s = 0, a_hgi = 0, a_egi = 0, a_egh = 0;
    double l_np = 0, l_qs = 0, l_rs = 0, l_or = 0, l_os = 0, l_lm = 0, l_il = 0, l_ik = 0, l_ek = 0, l_ae = 0, l_af = 0, l_ef = 0, l_ac = 0, l_gi = 0;

    double l_im = l;
    double l_mn = abs(x);
    double l_no = sqrt(y * y + z * z);
    double l_mo = sqrt(x * x + y * y + z * z);



    //-----求q1-----
    //求末端和向量n所在平面与yoz平面的交线的方程
    double v_m[3] =
    {
        y * c - z * b, z * a - x * c, x * b - y * a
    };

    double x_c = v_m[0];
    double y_c = v_m[1];
    double z_c = v_m[2];

    //求q1
    double k = -z_c / y_c;
    theta1 = atan(k);

    if (theta1 > 0 && theta1 < PI / 2)
    {
        q1 = PI / 2 - theta1;
    }
    else if (theta1 == 0)
    {
        if (z > 0)
        {
            q1 = -PI / 2;
        }
        else if (z < 0)
        {
            q1 = PI / 2;
        }
    }
    else if (theta1 > -PI / 2 && theta1 < 0)
    {
        q1 = -theta1 - PI / 2;
    }
    else if (theta1 == PI / 2 || theta1 == -PI / 2)
    {
        q1 = 0;
    }

    //-----q2-----
    //判断末端在交线的左边，还是右边
    if (theta1 > 0 && theta1 < PI / 2)
    {
        if (y - k > 0)
        {
            i = 1;  // 右边
        }
        else if (y - k < 0)
        {
            i = 0;//左边
        }
        else if (y - k == 0)
        {
            q2 = 0;
        }
    }
    else if (theta1 == 0)
    {
        if (q1 == PI / 2)
        {
            if (y > 0)
            {
                i = 1;
            }
            else if (y < 0)
            {
                i = 0;
            }
            else if (y == 0)
            {
                q2 = 0;
            }
        }
        else if (q1 == -PI / 2)
        {
            if (y > 0)
            {
                i = 0;
            }
            else if (y < 0)
            {
                i = 1;
            }
            else if (y == 0)
            {
                q2 = 0;
            }
        }
    }
    else if (theta1 > -PI / 2 && theta1 < 0)
    {
        if (y - k > 0)
        {
            i = 0;
        }
        else if (y - k < 0)
        {
            i = 1;
        }
        else if (y - k == 0)
        {
            q2 = 0;
        }
    }
    else if (theta1 == PI / 2 || theta1 == -PI / 2)
    {
        if (z > 0)
        {
            i = 0;
        }
        else if (z < 0)
        {
            i = 1;
        }
        else if (z == 0)
        {
            q2 = 0;
        }
    }

    //求q2
    if (i == 1)
    {
        if (theta1 > 0 && theta1 < PI / 2)
        {
            a_nop = theta1 - theta2;
        }
        else if (theta1 == 0)
        {
            a_nop = theta2;
        }
        else if (theta1 > -PI / 2 && theta1 < 0)
        {
            if (z > 0)
            {
                a_nop = theta1 + theta2;
            }
            else if (z < 0)
            {
                a_nop = PI + theta1 - theta2;
            }
            else if (z == 0)
            {
                a_nop = PI / 2 + theta1;
            }
        }
        else if (theta1 == PI / 2)
        {
            a_nop = PI / 2 - theta2;
        }

        l_np = l_no * sin(a_nop);
        a_mpn = atan(l_mn / l_np);

        if (x > 0)
        {
            q2 = PI / 2 - a_mpn;
        }
        else if (x < 0)
        {
            q2 = -(PI / 2 - a_mpn);
        }
        else if (x == 0)
        {
            if (c > 0)
            {
                q2 = PI / 2;
            }
            else if (c < 0)
            {
                q2 = -PI / 2;
            }
        }
    }
    else if (i == 0)
    {
        if (theta1 > 0 && theta1 < PI / 2)
        {
            if (z > 0)
            {
                a_nop = PI - theta1 - theta2;
            }
            else if (z < 0)
            {
                a_nop = theta2 - theta1;
            }
            else if (z == 0)
            {
                a_nop = PI / 2 - theta2;
            }
        }
        else if (theta1 == 0)
        {
            a_nop = theta2;
        }
        else if (theta1 > -PI / 2 && theta1 < 0)
        {
            a_nop = theta1 + theta2;
        }
        else if (theta1 == PI / 2)
        {
            a_nop = PI / 2 - theta2;
        }

        l_np = l_no * sin(a_nop);
        a_mpn = atan(l_mn / l_np);

        if (x > 0)
        {
            q2 = -(PI / 2 - a_mpn);
        }
        else if (x < 0)
        {
            q2 = PI / 2 - a_mpn;
        }
        else if (x == 0)
        {
            if (c > 0)
            {
                q2 = -PI / 2;
            }
            else if (c < 0)
            {
                q2 = PI / 2;
            }
        }
    }

    //-----q3-----
    //将末端坐标转换为腿平面坐标。
    //设腿上的四个转动副组成的腿平面为平面x0Oy0。
    //点O与原坐标一致，x0轴与x轴一致，
    //q1为0时，y0轴与y轴一致，q2不为0时，y0轴以y轴为轴，转动q2后的新轴。
    if (q2 == 0)
    {
        x0 = x;
        y0 = -sqrt(y * y + z * z);
    }
    else if (q2 != 0)
    {
        x0 = l_mn / sin(a_mpn) * x / abs(x);
        y0 = -sqrt(l_no * l_no - l_np * l_np);
    }

    //求脚踝转动副坐标（x1，y1）
    l_qs = a / cos(q2);
    l_rs = sqrt(l_qs * l_qs - a * a);
    l_or = sqrt(b * b + c * c);
    l_os = sqrt(l_or * l_or - l_rs * l_rs);
    a_lim = atan(l_os / l_qs);
    a_eij = atan(kLEJ / kLIJ);
    l_lm = l_im * sin(a_lim);
    l_il = l_im * cos(a_lim);
    theta2 = acos(abs(z) / l_no);

    if (abs(theta1) == PI / 2)
    {
        if (b > 0)
        {
            if (a_eij + a_lim > PI / 2)
            {
                a_eik = PI - a_eij - a_lim;
                l_ik = kLEI * cos(a_eik);
                l_ek = kLEI * sin(a_eik);

                x1 = x0 - l_ik - l_il;
                y1 = y0 - l_lm + l_ek;
            }
            else if (a_eij + a_lim == PI / 2)
            {
                x1 = x0 - l_il;
                y1 = y0 - l_lm + kLEI;
            }
            else if (a_eij + a_lim < PI / 2)
            {
                a_eik = a_eij + a_lim;
                l_ik = kLEI * cos(a_eik);
                l_ek = kLEI * sin(a_eik);

                x1 = x0 + l_ik - l_il;
                y1 = y0 - l_lm + l_ek;
            }
        }
        else if (b == 0)
        {
            x1 = x0 - l_im + kLIJ;
            y1 = y0 + kLEJ;
        }
        else if (b < 0)
        {
            a_eik = a_eij - a_lim;
            l_ik = kLEI * cos(a_eik);
            l_ek = kLEI * sin(a_eik);

            x1 = x0 + l_ik - l_il;
            y1 = y0 + l_lm + l_ek;
        }
    }
    else if (-k * c < b)
    {
        if (a_eij + a_lim > PI / 2)
        {
            a_eik = PI - a_eij - a_lim;
            l_ik = kLEI * cos(a_eik);
            l_ek = kLEI * sin(a_eik);

            x1 = x0 - l_ik - l_il;
            y1 = y0 - l_lm + l_ek;
        }
        else if (a_eij + a_lim == PI / 2)
        {
            x1 = x0 - l_il;
            y1 = y0 - l_lm + kLEI;
        }
        else if (a_eij + a_lim < PI / 2)
        {
            a_eik = a_eij + a_lim;
            l_ik = kLEI * cos(a_eik);
            l_ek = kLEI * sin(a_eik);

            x1 = x0 + l_ik - l_il;
            y1 = y0 - l_lm + l_ek;
        }
    }
    else if (-k * c == b)
    {
        x1 = x0 - l_im + kLIJ;
        y1 = y0 + kLEJ;
    }
    else if (-k * c > b)
    {
        a_eik = a_eij - a_lim;
        l_ik = kLEI * cos(a_eik);
        l_ek = kLEI * sin(a_eik);

        x1 = x0 + l_ik - l_il;
        y1 = y0 + l_lm + l_ek;
    }

    //求q3
    l_ae = sqrt(x1 * x1 + (y1 + kLAO) * (y1 + kLAO));
    l_af = -(y1 + kLAO);
    l_ef = abs(x1);
    a_cbe = acos((kLBC * kLBC + kLBE * kLBE - kLCE * kLCE) / (2 * kLBC * kLBE));
    a_abe = acos((kLAB * kLAB + kLBE * kLBE - l_ae * l_ae) / (2 * kLAB * kLBE));
    a_abc = a_abe - a_cbe;
    l_ac = sqrt(kLAB * kLAB + kLBC * kLBC - 2 * kLAB * kLBC * cos(a_abc));
    a_bac = acos((kLAB * kLAB + l_ac * l_ac - kLBC * kLBC) / (2 * kLAB * l_ac));
    a_cad = acos((l_ac * l_ac + kLAD * kLAD - kLCD * kLCD) / (2 * l_ac * kLAD));
    a_bad = a_bac + a_cad;
    a_bae = acos((kLAB * kLAB + l_ae * l_ae - kLBE * kLBE) / (2 * kLAB * l_ae));
    a_eaf = acos((l_ae * l_ae + l_af * l_af - l_ef * l_ef) / (2 * l_ae * l_af));

    if (x1 > 0)
    {
        a_daf = a_bad - a_bae + a_eaf;
    }
    else if (x1 == 0)
    {
        a_daf = a_bad - a_bae;
    }
    else if (x1 < 0)
    {
        a_daf = a_bad - a_bae - a_eaf;
    }

    q3 = PI - a_daf - kQ30;

    //-----q4-----
    a_adc = acos((kLAD * kLAD + kLCD * kLCD - l_ac * l_ac) / (2 * kLAD * kLCD));

    q4 = a_adc - 3.0 / 4.0 * PI;

    //-----q5-----
    //求∠GEI
    a_aec = acos((l_ae * l_ae + kLCE * kLCE - l_ac * l_ac) / (2 * l_ae * kLCE));
    a_aeb = acos((l_ae * l_ae + kLBE * kLBE - kLAB * kLAB) / (2 * l_ae * kLBE));
    a_aef = acos((l_ae * l_ae + x1 * x1 - (y1 + kLAO) * (y1 + kLAO)) / (2 * l_ae * abs(x1)));
    a_bec = acos((kLCE * kLCE + kLBE * kLBE - kLBC * kLBC) / (2 * kLCE * kLBE));

    if (x1 > 0)
    {
        if (a_aeb > a_bec)
        {
            theta3 = a_aef - (a_aeb - a_bec);
        }
        else if (a_aeb == a_bec)
        {
            theta3 = a_aef;
        }
        else if (a_aeb < a_bec)
        {
            theta3 = a_aef + (a_bec - a_aeb);
        }

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }
    else if (x1 == 0)
    {
        if (a_aeb > a_bec)
        {
            theta3 = PI / 2 - (a_aeb - a_bec);
        }
        else if (a_aeb == a_bec)
        {
            theta3 = PI / 2;
        }
        else if (a_aeb < a_bec)
        {
            theta3 = PI / 2 - (a_aeb - a_bec);
        }

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }
    else if (x1 < 0)
    {
        theta3 = PI - a_aef - a_aeb + a_bec;

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }

    a_gei = theta3 + theta4;

    //求∠GEI最大值和最小值
    a_gei_b = acos((kLEG * kLEG + kLEI * kLEI - (kLGH + kLHI) * (kLGH + kLHI)) / (2 * kLEG * kLEI));
    a_gei_s = acos((kLEG * kLEG + (kLEI + kLHI) * (kLEI + kLHI) - kLGH * kLGH) / (2 * kLEG * (kLEI + kLHI)));

    //求q5
    if (a_gei > a_gei_b)
    {
        a_gei = a_gei_b;
    }
    else if (a_gei < a_gei_s)
    {
        a_gei = a_gei_s;
    }

    l_gi = sqrt(kLEG * kLEG + kLEI * kLEI - 2 * kLEG * kLEI * cos(a_gei));
    a_hgi = acos((kLGH * kLGH + l_gi * l_gi - kLHI * kLHI) / (2 * kLGH * l_gi));
    a_egi = acos((kLEG * kLEG + l_gi * l_gi - kLEI * kLEI) / (2 * kLEG * l_gi));
    a_egh = a_egi + a_hgi;

    q5 = a_egh - kQ50;

    //输出
    motion_pos[0] = q1;
    motion_pos[1] = q2;
    motion_pos[2] = q3;
    motion_pos[3] = q4;
    motion_pos[4] = q5;


}

void ikForBipedRobotforTest(double x, double y, double z, double a, double b, double c, double l, double input[6]) {
    //-----已知数据-----
    //L、l为长
    //v为向量
    //a为角度
    const double kLAO = 154.5;
    const double kLAD = 150;
    const double kLCD = 422.02;
    const double kLAB = 487.46;
    const double kLBC = 88.82;
    const double kLBE = 509.51;
    const double kLCE = 426.61;
    const double kLEG = 326.82;
    const double kLHI = 280;
    const double kLGH = 120;
    const double kLIJ = 85;
    const double kLEJ = 40;
    const double kLEI = sqrt(kLIJ * kLIJ + kLEJ * kLEJ);
    const double kQ30 = 2.768114255522946;
    const double kQ50 = 1.020376369330288;

    double v_om[3] =
    {
        x, y, z
    };
    double v_vn[3] =
    {
        a, b, c
    };

    //-----计算数据-----
    double q1 = 0, q2 = 0, q3 = 0, q4 = 0, q5 = 0;
    double i = 0;//判断左右，0左1右
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    double theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0;
    double a_nop = 0, a_mpn = 0, a_lim = 0, a_eij = 0, a_eik = 0, a_cbe = 0, a_abe = 0, a_abc = 0, a_bac = 0, a_cad = 0, a_bad = 0, a_bae = 0, a_eaf = 0,
        a_daf = 0, a_adc = 0, a_aec = 0, a_aeb = 0, a_aef = 0, a_bec = 0, a_gei = 0, a_gei_b = 0, a_gei_s = 0, a_hgi = 0, a_egi = 0, a_egh = 0;
    double l_np = 0, l_qs = 0, l_rs = 0, l_or = 0, l_os = 0, l_lm = 0, l_il = 0, l_ik = 0, l_ek = 0, l_ae = 0, l_af = 0, l_ef = 0, l_ac = 0, l_gi = 0;

    double l_im = l;
    double l_mn = std::abs(x);
    double l_no = std::sqrt(y * y + z * z);
    double l_mo = std::sqrt(x * x + y * y + z * z);



    //-----求q1-----
    //求末端和向量n所在平面与yoz平面的交线的方程
    double v_m[3] =
    {
        y * c - z * b, z * a - x * c, x * b - y * a
    };

    double x_c = v_m[0];
    double y_c = v_m[1];
    double z_c = v_m[2];

    //求q1
    double k = -z_c / y_c;
    theta1 = std::atan(k);

    if (theta1 > 0 && theta1 < PI / 2)
    {
        q1 = PI / 2 - theta1;
    }
    else if (theta1 == 0)
    {
        if (z > 0)
        {
            q1 = -PI / 2;
        }
        else if (z < 0)
        {
            q1 = PI / 2;
        }
    }
    else if (theta1 > -PI / 2 && theta1 < 0)
    {
        q1 = -theta1 - PI / 2;
    }
    else if (theta1 == PI / 2 || theta1 == -PI / 2)
    {
        q1 = 0;
    }

    //-----q2-----
    //判断末端在交线的左边，还是右边
    if (theta1 > 0 && theta1 < PI / 2)
    {
        if (y - k > 0)
        {
            i = 1;  // 右边
        }
        else if (y - k < 0)
        {
            i = 0;//左边
        }
        else if (y - k == 0)
        {
            q2 = 0;
        }
    }
    else if (theta1 == 0)
    {
        if (q1 == PI / 2)
        {
            if (y > 0)
            {
                i = 1;
            }
            else if (y < 0)
            {
                i = 0;
            }
            else if (y == 0)
            {
                q2 = 0;
            }
        }
        else if (q1 == -PI / 2)
        {
            if (y > 0)
            {
                i = 0;
            }
            else if (y < 0)
            {
                i = 1;
            }
            else if (y == 0)
            {
                q2 = 0;
            }
        }
    }
    else if (theta1 > -PI / 2 && theta1 < 0)
    {
        if (y - k > 0)
        {
            i = 0;
        }
        else if (y - k < 0)
        {
            i = 1;
        }
        else if (y - k == 0)
        {
            q2 = 0;
        }
    }
    else if (theta1 == PI / 2 || theta1 == -PI / 2)
    {
        if (z > 0)
        {
            i = 0;
        }
        else if (z < 0)
        {
            i = 1;
        }
        else if (z == 0)
        {
            q2 = 0;
        }
    }

    //求q2
    if (i == 1)
    {
        if (theta1 > 0 && theta1 < PI / 2)
        {
            a_nop = theta1 - theta2;
        }
        else if (theta1 == 0)
        {
            a_nop = theta2;
        }
        else if (theta1 > -PI / 2 && theta1 < 0)
        {
            if (z > 0)
            {
                a_nop = theta1 + theta2;
            }
            else if (z < 0)
            {
                a_nop = PI + theta1 - theta2;
            }
            else if (z == 0)
            {
                a_nop = PI / 2 + theta1;
            }
        }
        else if (theta1 == PI / 2)
        {
            a_nop = PI / 2 - theta2;
        }

        l_np = l_no * std::sin(a_nop);
        a_mpn = std::atan(l_mn / l_np);

        if (x > 0)
        {
            q2 = PI / 2 - a_mpn;
        }
        else if (x < 0)
        {
            q2 = -(PI / 2 - a_mpn);
        }
        else if (x == 0)
        {
            if (c > 0)
            {
                q2 = PI / 2;
            }
            else if (c < 0)
            {
                q2 = -PI / 2;
            }
        }
    }
    else if (i == 0)
    {
        if (theta1 > 0 && theta1 < PI / 2)
        {
            if (z > 0)
            {
                a_nop = PI - theta1 - theta2;
            }
            else if (z < 0)
            {
                a_nop = theta2 - theta1;
            }
            else if (z == 0)
            {
                a_nop = PI / 2 - theta2;
            }
        }
        else if (theta1 == 0)
        {
            a_nop = theta2;
        }
        else if (theta1 > -PI / 2 && theta1 < 0)
        {
            a_nop = theta1 + theta2;
        }
        else if (theta1 == PI / 2)
        {
            a_nop = PI / 2 - theta2;
        }

        l_np = l_no * sin(a_nop);
        a_mpn = atan(l_mn / l_np);

        if (x > 0)
        {
            q2 = -(PI / 2 - a_mpn);
        }
        else if (x < 0)
        {
            q2 = PI / 2 - a_mpn;
        }
        else if (x == 0)
        {
            if (c > 0)
            {
                q2 = -PI / 2;
            }
            else if (c < 0)
            {
                q2 = PI / 2;
            }
        }
    }

    //-----q3-----
    //将末端坐标转换为腿平面坐标。
    //设腿上的四个转动副组成的腿平面为平面x0Oy0。
    //点O与原坐标一致，x0轴与x轴一致，
    //q1为0时，y0轴与y轴一致，q2不为0时，y0轴以y轴为轴，转动q2后的新轴。
    if (q2 == 0)
    {
        x0 = x;
        y0 = -std::sqrt(y * y + z * z);
    }
    else if (q2 != 0)
    {
        x0 = l_mn / std::sin(a_mpn) * x / std::abs(x);
        y0 = -std::sqrt(l_no * l_no - l_np * l_np);
    }

    //求脚踝转动副坐标（x1，y1）
    l_qs = a / std::cos(q2);
    l_rs = std::sqrt(l_qs * l_qs - a * a);
    l_or = std::sqrt(b * b + c * c);
    l_os = std::sqrt(l_or * l_or - l_rs * l_rs);
    a_lim = std::atan(l_os / l_qs);
    a_eij = std::atan(kLEJ / kLIJ);
    l_lm = l_im * std::sin(a_lim);
    l_il = l_im * std::cos(a_lim);
    theta2 = std::acos(std::abs(z) / l_no);

    if (std::abs(theta1) == PI / 2)
    {
        if (b > 0)
        {
            if (a_eij + a_lim > PI / 2)
            {
                a_eik = PI - a_eij - a_lim;
                l_ik = kLEI * std::cos(a_eik);
                l_ek = kLEI * std::sin(a_eik);

                x1 = x0 - l_ik - l_il;
                y1 = y0 - l_lm + l_ek;
            }
            else if (a_eij + a_lim == PI / 2)
            {
                x1 = x0 - l_il;
                y1 = y0 - l_lm + kLEI;
            }
            else if (a_eij + a_lim < PI / 2)
            {
                a_eik = a_eij + a_lim;
                l_ik = kLEI * std::cos(a_eik);
                l_ek = kLEI * std::sin(a_eik);

                x1 = x0 + l_ik - l_il;
                y1 = y0 - l_lm + l_ek;
            }
        }
        else if (b == 0)
        {
            x1 = x0 - l_im + kLIJ;
            y1 = y0 + kLEJ;
        }
        else if (b < 0)
        {
            a_eik = a_eij - a_lim;
            l_ik = kLEI * std::cos(a_eik);
            l_ek = kLEI * std::sin(a_eik);

            x1 = x0 + l_ik - l_il;
            y1 = y0 + l_lm + l_ek;
        }
    }
    else if (-k * c < b)
    {
        if (a_eij + a_lim > PI / 2)
        {
            a_eik = PI - a_eij - a_lim;
            l_ik = kLEI * std::cos(a_eik);
            l_ek = kLEI * std::sin(a_eik);

            x1 = x0 - l_ik - l_il;
            y1 = y0 - l_lm + l_ek;
        }
        else if (a_eij + a_lim == PI / 2)
        {
            x1 = x0 - l_il;
            y1 = y0 - l_lm + kLEI;
        }
        else if (a_eij + a_lim < PI / 2)
        {
            a_eik = a_eij + a_lim;
            l_ik = kLEI * std::cos(a_eik);
            l_ek = kLEI * std::sin(a_eik);

            x1 = x0 + l_ik - l_il;
            y1 = y0 - l_lm + l_ek;
        }
    }
    else if (-k * c == b)
    {
        x1 = x0 - l_im + kLIJ;
        y1 = y0 + kLEJ;
    }
    else if (-k * c > b)
    {
        a_eik = a_eij - a_lim;
        l_ik = kLEI * std::cos(a_eik);
        l_ek = kLEI * std::sin(a_eik);

        x1 = x0 + l_ik - l_il;
        y1 = y0 + l_lm + l_ek;
    }

    //求q3
    l_ae = std::sqrt(x1 * x1 + (y1 + kLAO) * (y1 + kLAO));
    l_af = -(y1 + kLAO);
    l_ef = std::abs(x1);
    a_cbe = std::acos((kLBC * kLBC + kLBE * kLBE - kLCE * kLCE) / (2 * kLBC * kLBE));
    a_abe = std::acos((kLAB * kLAB + kLBE * kLBE - l_ae * l_ae) / (2 * kLAB * kLBE));
    a_abc = a_abe - a_cbe;
    l_ac = std::sqrt(kLAB * kLAB + kLBC * kLBC - 2 * kLAB * kLBC * std::cos(a_abc));
    a_bac = std::acos((kLAB * kLAB + l_ac * l_ac - kLBC * kLBC) / (2 * kLAB * l_ac));
    a_cad = std::acos((l_ac * l_ac + kLAD * kLAD - kLCD * kLCD) / (2 * l_ac * kLAD));
    a_bad = a_bac + a_cad;
    a_bae = std::acos((kLAB * kLAB + l_ae * l_ae - kLBE * kLBE) / (2 * kLAB * l_ae));
    a_eaf = std::acos((l_ae * l_ae + l_af * l_af - l_ef * l_ef) / (2 * l_ae * l_af));

    if (x1 > 0)
    {
        a_daf = a_bad - a_bae + a_eaf;
    }
    else if (x1 == 0)
    {
        a_daf = a_bad - a_bae;
    }
    else if (x1 < 0)
    {
        a_daf = a_bad - a_bae - a_eaf;
    }

    q3 = PI - a_daf - kQ30;

    //-----q4-----
    a_adc = std::acos((kLAD * kLAD + kLCD * kLCD - l_ac * l_ac) / (2 * kLAD * kLCD));

    q4 = a_adc - 3.0 / 4.0 * PI;

    //-----q5-----
    //求∠GEI
    a_aec = std::acos((l_ae * l_ae + kLCE * kLCE - l_ac * l_ac) / (2 * l_ae * kLCE));
    a_aeb = std::acos((l_ae * l_ae + kLBE * kLBE - kLAB * kLAB) / (2 * l_ae * kLBE));
    a_aef = std::acos((l_ae * l_ae + x1 * x1 - (y1 + kLAO) * (y1 + kLAO)) / (2 * l_ae * std::abs(x1)));
    a_bec = std::acos((kLCE * kLCE + kLBE * kLBE - kLBC * kLBC) / (2 * kLCE * kLBE));

    if (x1 > 0)
    {
        if (a_aeb > a_bec)
        {
            theta3 = a_aef - (a_aeb - a_bec);
        }
        else if (a_aeb == a_bec)
        {
            theta3 = a_aef;
        }
        else if (a_aeb < a_bec)
        {
            theta3 = a_aef + (a_bec - a_aeb);
        }

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }
    else if (x1 == 0)
    {
        if (a_aeb > a_bec)
        {
            theta3 = PI / 2 - (a_aeb - a_bec);
        }
        else if (a_aeb == a_bec)
        {
            theta3 = PI / 2;
        }
        else if (a_aeb < a_bec)
        {
            theta3 = PI / 2 - (a_aeb - a_bec);
        }

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }
    else if (x1 < 0)
    {
        theta3 = PI - a_aef - a_aeb + a_bec;

        if (a_lim == 0)
        {
            theta4 = a_eij;
        }
        else if (a_lim != 0)
        {
            if (b > 0)
            {
                theta4 = a_eij + a_lim;
            }
            else if (b < 0)
            {
                theta4 = a_eij - a_lim;
            }
        }
    }

    a_gei = theta3 + theta4;

    //求∠GEI最大值和最小值
    a_gei_b = std::acos((kLEG * kLEG + kLEI * kLEI - (kLGH + kLHI) * (kLGH + kLHI)) / (2 * kLEG * kLEI));
    a_gei_s = std::acos((kLEG * kLEG + (kLEI + kLHI) * (kLEI + kLHI) - kLGH * kLGH) / (2 * kLEG * (kLEI + kLHI)));

    //求q5
    if (a_gei > a_gei_b)
    {
        a_gei = a_gei_b;
    }
    else if (a_gei < a_gei_s)
    {
        a_gei = a_gei_s;
    }

    l_gi = std::sqrt(kLEG * kLEG + kLEI * kLEI - 2 * kLEG * kLEI * std::cos(a_gei));
    a_hgi = std::acos((kLGH * kLGH + l_gi * l_gi - kLHI * kLHI) / (2 * kLGH * l_gi));
    a_egi = std::acos((kLEG * kLEG + l_gi * l_gi - kLEI * kLEI) / (2 * kLEG * l_gi));
    a_egh = a_egi + a_hgi;

    q5 = a_egh - kQ50;

    //输出
    input[0] = q1;
    input[1] = q2;
    input[2] = q3;
    input[3] = q4;
    input[4] = q5;
    input[5] = a_adc;


}

auto inverseCalculation(double* leg_in_ground, double* body_in_ground, double* end_pointing, double* end_position_on_foot, double* input)->int
{
    double real_pm1[16] = { 0 }, real_pm2[16] = { 0 };
    s_pm_dot_inv_pm(cs_of_leg1, body_in_ground, real_pm1);
    s_pm_dot_inv_pm(cs_of_leg2, body_in_ground, real_pm2);

    double xyz_in_leg[6] = { 0 }; //腿末端在腿坐标系下的表达
    s_pp2pp(real_pm1, leg_in_ground + 0 * 3, xyz_in_leg + 0 * 3);
    s_pp2pp(real_pm2, leg_in_ground + 1 * 3, xyz_in_leg + 1 * 3);

    ikForBipedRobot(xyz_in_leg + 0 * 3, end_pointing + 0 * 3, end_position_on_foot + 0, input + 0 * 5);
    ikForBipedRobot(xyz_in_leg + 1 * 3, end_pointing + 1 * 3, end_position_on_foot + 1, input + 1 * 5);

    return 0;
}