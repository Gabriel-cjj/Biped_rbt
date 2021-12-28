#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>

#include "robot.h"
#include "plan.h"

#include "kinematics.h"

extern double foot_position_start_point[6];

double current[10] = { 0 };
double input_angle[10] = { 0 };
double prepare_position_[10] =
{
    (90 - 74.1) / 180.0 * PI,
    -(180 - 126.85) / 180.0 * PI,
    (180 - 151.92) / 180.0 * PI,
    0,
    0,
    0,
    0,
    (180 - 151.92) / 180.0 * PI,
    -(180 - 126.85) / 180.0 * PI,
    (90 - 74.1) / 180.0 * PI
};

double current_body_and_leg[26] = {
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1,
};

//输出参数，模型曲线测试使用
extern double file_current_leg[6];
extern double file_current_body[16];
extern double time_test;
extern double cs_of_leg1[16];
extern double cs_of_leg2[16];



using namespace aris::dynamic;
using namespace aris::plan;
extern const double PI;
namespace robot
{

//修正读数
auto PositionCheck::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto PositionCheck::executeRT()->int
{
    static double begin_angle[10];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();
    }

    if (begin_angle[6] > 0)
    {
        begin_angle[6] = begin_angle[6] - 2 * PI;
        controller()->motionPool()[6].setPosOffset(-2 * PI);
    }

    if (begin_angle[7] < 0)
    {
        begin_angle[7] = begin_angle[7] + 2 * PI;
        controller()->motionPool()[7].setPosOffset(-2 * PI);
    }

    if (begin_angle[9] < 0)
    {
        begin_angle[9] = begin_angle[9] + 2 * PI;
        controller()->motionPool()[9].setPosOffset(-2 * PI);
    }

    return 0;
}
auto PositionCheck::collectNrt()->void {}
PositionCheck::PositionCheck(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"check_pos\"/>");
}


//检查使能情况
//全体电机移动一段距离，使能成功则可以移动。
auto CheckEnable::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto CheckEnable::executeRT()->int
{
    static double begin_angle[10];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();
    }

    TCurve s1(0.016, 0.3);
    s1.getCurveParam();

    for (int i = 0; i < 10; i++)
    {
        controller()->motionPool()[i].setTargetPos(begin_angle[i] + dir_ * s1.getTCurve(count()));
    }

    return s1.getTc() * 1000 - count();
}
auto CheckEnable::collectNrt()->void {}
CheckEnable::CheckEnable(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"check_enable\">"
        "		<Param name=\"direction\" default=\"0.2\" abbreviation=\"d\"/>"
        "</Command>");
}

//机器人准备
//回到最初设置的位置
auto RobotPrepare::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto RobotPrepare::executeRT()->int
{
    static double begin_angle[10];
    double angle[10] = { 0 };
    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();
    }

    TCurve s1(0.016, 0.3);
    s1.getCurveParam();

    for (int i = 0; i < 10; i++)
    {
        angle[i] = begin_angle[i] + (prepare_position_[i] - begin_angle[i]) * s1.getTCurve(count());
    }

    for (int i = 0; i < 10; i++)
    {
        controller()->motionPool()[i].setTargetPos(angle[i]);
    }

    return s1.getTc() * 1000 - count();
}
auto RobotPrepare::collectNrt()->void {}
RobotPrepare::RobotPrepare(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"rbt_prepare\"/>");
}

//读取当前位置
auto ReadPosition::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto ReadPosition::executeRT()->int
{
    static double begin_angle[10];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();
    }

    for (int i = 0; i < 10; i++)
    {
        mout() << i << " " <<begin_angle[i] << "\t\n";
    }

    return 0;
}
auto ReadPosition::collectNrt()->void {}
ReadPosition::ReadPosition(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"read_p\"/>");
}

//读取电流
auto ReadCurrent::prepareNrt()->void
{
    
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto ReadCurrent::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("actual_current");

    if (count() == 1)ret = 10000;

    ret = ret - 1;

    for (int i = 0; i < 10; i++)
    {
        current[i] = this->ecController()->motionPool()[i].readPdo(0x6077, 0x00, current[i]);
        lout() << current[i] << "\t";
    }

    return ret;
}
auto ReadCurrent::collectNrt()->void {}
ReadCurrent::ReadCurrent(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"read_c\"/>");
}

//原地踏步
auto WalkStep::prepareNrt()->void
{
    step_ = doubleParam("step");
    v_ = doubleParam("speed");
    h_ = doubleParam("high");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto WalkStep::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("walkStep");
    static double begin_angle[10];
    int ret = 1;

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();

        {
            lout() << "q1_l" << "\t";
            lout() << "q2_l" << "\t";
            lout() << "q3_l" << "\t";
            lout() << "q4_l" << "\t";
            lout() << "q5_l" << "\t";
            lout() << "q1_r" << "\t";
            lout() << "q2_r" << "\t";
            lout() << "q3_r" << "\t";
            lout() << "q4_r" << "\t";
            lout() << "q5_r" << "\t";
            lout() << "time" << std::endl;
        }


    }

    TCurve s1(v_ / 10, v_);
    s1.getCurveParam();
    EllipseTrajectory e1(0, h_, 0, s1);

    //步态规划
    ret = forwardBipedRobot(step_, count() - 1, &e1, input_angle);

    if (count() == 1)
    {
        mout() << time_test << "\n";
        std::cout << s1.getTc() << std::endl;
    }



    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 10; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        ////输出身体和足尖曲线
        //for (int j = 0; j < 6; j++)
        //{
        //    lout() << file_current_leg[j] << "\t";
        //}

        //lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;

        ////输出电流
        //for (int i = 0; i < 10; i++)
        //{
        //    current[i] = this->ecController()->motionPool()[i].readPdo(0x6077, 0x00, current[i]);
        //    lout() << current[i] << "\t" << std::endl;
        //}

        lout() << std::endl;

    }

    double angle0 = begin_angle[0] + input_angle[4];
    double angle1 = begin_angle[1] + input_angle[3];
    double angle2 = begin_angle[2] + input_angle[2];
    double angle3 = begin_angle[3] + input_angle[1];
    double angle4 = begin_angle[4] + input_angle[0];
    double angle5 = begin_angle[5] + input_angle[5];
    double angle6 = begin_angle[6] + input_angle[6];
    double angle7 = begin_angle[7] + input_angle[7];
    double angle8 = begin_angle[8] + input_angle[8];
    double angle9 = begin_angle[9] + input_angle[9];

    double input_angle_model[10] =
    {
        angle0, angle1, angle2, angle3, angle4,
        angle5, angle6, angle7, angle8, angle9
    };
    
    //model()->getInputPos(input_angle_model);

    //model()->setTime(0.001 * count());
    

    //发送电机角度
    //controller()->motionPool()[0].setTargetPos(angle0);
    //controller()->motionPool()[1].setTargetPos(angle1);
    //controller()->motionPool()[2].setTargetPos(angle2);
    //controller()->motionPool()[3].setTargetPos(angle3);
    //controller()->motionPool()[4].setTargetPos(angle4);
    //controller()->motionPool()[5].setTargetPos(angle5);
    //controller()->motionPool()[6].setTargetPos(angle6);
    //controller()->motionPool()[7].setTargetPos(angle7);
    //controller()->motionPool()[8].setTargetPos(angle8);
    //controller()->motionPool()[9].setTargetPos(angle9);

    if (ret == 0)std::cout << count() << std::endl;
    return ret;
}
auto WalkStep::collectNrt()->void {}
WalkStep::WalkStep(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"walk_step\">"
        "<GroupParam>"
        "       <Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "		<Param name=\"speed\" default=\"0.3\" abbreviation=\"v\"/>"
        "		<Param name=\"high\" default=\"100\" abbreviation=\"h\"/>"
        "</GroupParam>"
        "</Command>");
}

//测试电机
auto TestMotor::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    motornumber_ = doubleParam("motor_number");

    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto TestMotor::executeRT()->int
{
    static double begin_angle[10];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();
    }

    TCurve s1(0.016, 0.3);
    s1.getCurveParam();
    double angle0 = begin_angle[motornumber_] + dir_ * s1.getTCurve(count());


    controller()->motionPool()[motornumber_].setTargetPos(angle0);
    return s1.getTc() * 1000 - count();
}
auto TestMotor::collectNrt()->void {}
TestMotor::TestMotor(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"test_motor\">"
        "<GroupParam>"
        "		<Param name=\"direction\" default=\"0.3\" abbreviation=\"d\"/>"
        "		<Param name=\"motor_number\" default=\"0\" abbreviation=\"m\"/>"
        "</GroupParam>"
        "</Command>");
}

//末端位置移动
auto MoveEnd::prepareNrt()->void
{
    x1_ = doubleParam("x_left_leg");
    y1_ = doubleParam("y_left_leg");
    z1_ = doubleParam("z_left_leg");
    x2_ = doubleParam("x_right_leg");
    y2_ = doubleParam("y_right_leg");
    z2_ = doubleParam("z_right_leg");
    l1_ = doubleParam("left_end_position_on_foot");
    l2_ = doubleParam("right_end_position_on_foot");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveEnd::executeRT()->int
{
    static double begin_angle[10];
    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();

    }

    TCurve s1(0.016, 0.3);
    s1.getCurveParam();

    ikForBipedRobotforTest(x1_, y1_, z1_, 1, 0, 0, l1_, input_angle + 0 * 5);
    ikForBipedRobotforTest(x2_, y2_, z2_, 1, 0, 0, l2_, input_angle + 1 * 5);

    foot_position_start_point[0] = x1_ - kBodyLong + 82.54606669;
    foot_position_start_point[1] = y1_;
    foot_position_start_point[2] = z1_ - kBodyWidth / 2;
    foot_position_start_point[3] = x2_ - kBodyLong + 82.54606669;
    foot_position_start_point[4] = y2_;
    foot_position_start_point[5] = z2_ + kBodyWidth / 2;

    double angle0 = begin_angle[0] + input_angle[4] * s1.getTCurve(count());
    double angle1 = begin_angle[1] + input_angle[3] * s1.getTCurve(count());
    double angle2 = begin_angle[2] + input_angle[2] * s1.getTCurve(count());
    double angle3 = begin_angle[3] + input_angle[1] * s1.getTCurve(count());
    double angle4 = begin_angle[4] + input_angle[0] * s1.getTCurve(count());
    double angle5 = begin_angle[5] + input_angle[5] * s1.getTCurve(count());
    double angle6 = begin_angle[6] + input_angle[6] * s1.getTCurve(count());
    double angle7 = begin_angle[7] + input_angle[7] * s1.getTCurve(count());
    double angle8 = begin_angle[8] + input_angle[8] * s1.getTCurve(count());
    double angle9 = begin_angle[9] + input_angle[9] * s1.getTCurve(count());

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 10; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 6; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }

        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11];

        //输出电流
        for (int i = 0; i < 10; i++)
        {
            current[i] = this->ecController()->motionPool()[i].readPdo(0x6077, 0x00, current[i]);
            lout() << current[i] << "\t" << std::endl;
        }

    }


    //发送电机角度
    controller()->motionPool()[0].setTargetPos(angle0);
    controller()->motionPool()[1].setTargetPos(angle1);
    controller()->motionPool()[2].setTargetPos(angle2);
    controller()->motionPool()[3].setTargetPos(angle3);
    controller()->motionPool()[4].setTargetPos(angle4);
    controller()->motionPool()[5].setTargetPos(angle5);
    controller()->motionPool()[6].setTargetPos(angle6);
    controller()->motionPool()[7].setTargetPos(angle7);
    controller()->motionPool()[8].setTargetPos(angle8);
    controller()->motionPool()[9].setTargetPos(angle9);

    return s1.getTc() * 1000 - count();
}
auto MoveEnd::collectNrt()->void {}
MoveEnd::MoveEnd(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"move_end\">"
        "<GroupParam>"
        "       <Param name=\"x_left_leg\" default=\"0\" abbreviation=\"x\"/>"
        "		<Param name=\"y_left_leg\" default=\"-1131.9101101572\" abbreviation=\"y\"/>"
        "		<Param name=\"z_left_leg\" default=\"0\" abbreviation=\"z\"/>"
        "       <Param name=\"x_right_leg\" default=\"0\" abbreviation=\"X\"/>"
        "		<Param name=\"y_right_leg\" default=\"-1131.9101101572\" abbreviation=\"Y\"/>"
        "		<Param name=\"z_right_leg\" default=\"0\" abbreviation=\"Z\"/>"
        "		<Param name=\"left_end_position_on_foot\" default=\"85\" abbreviation=\"l\"/>"
        "		<Param name=\"right_end_position_on_foot\" default=\"85\" abbreviation=\"L\"/>"
        "</GroupParam>"
        "</Command>");

    //"<Command name=\"move_end\"/>");
}

//前进
auto WalkForward::prepareNrt()->void
{
    step_ = doubleParam("step");
    v_ = doubleParam("speed");
    h_ = doubleParam("high");
    l_ = doubleParam("long");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto WalkForward::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("WalkForward");
    static double begin_angle[10]{ 0,0,0,0,0,0,0,0,0,0 };
    int ret = 1;

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();

    }

    TCurve s1(v_ / 10, v_);
    s1.getCurveParam();
    EllipseTrajectory e1(l_, h_, 0, s1);

    //步态规划
    ret = forwardBipedRobot(step_, count() - 1, &e1, input_angle);

    if (count() == 1)
    {
        mout() << time_test << "\n";
        std::cout << s1.getTc() << std::endl;
    }



    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 10; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 6; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }

        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;

        ////输出电流
        //for (int i = 0; i < 10; i++)
        //{
        //    current[i] = this->ecController()->motionPool()[i].readPdo(0x6077, 0x00, current[i]);
        //    lout() << current[i] << "\t" << std::endl;
        //}

    }

    double angle0 = begin_angle[0] + input_angle[4];
    double angle1 = begin_angle[1] + input_angle[3];
    double angle2 = begin_angle[2] + input_angle[2];
    double angle3 = begin_angle[3] + input_angle[1];
    double angle4 = begin_angle[4] + input_angle[0];
    double angle5 = begin_angle[5] + input_angle[5];
    double angle6 = begin_angle[6] + input_angle[6];
    double angle7 = begin_angle[7] + input_angle[7];
    double angle8 = begin_angle[8] + input_angle[8];
    double angle9 = begin_angle[9] + input_angle[9];

    double input_angle_model[10] =
    {
        angle0, angle1, angle2, angle3, angle4,
        angle5, angle6, angle7, angle8, angle9
    };

    //model()->getInputPos(input_angle_model);

    //model()->setTime(0.001 * count());


    //发送电机角度
    controller()->motionPool()[0].setTargetPos(angle0);
    controller()->motionPool()[1].setTargetPos(angle1);
    controller()->motionPool()[2].setTargetPos(angle2);
    controller()->motionPool()[3].setTargetPos(angle3);
    controller()->motionPool()[4].setTargetPos(angle4);
    controller()->motionPool()[5].setTargetPos(angle5);
    controller()->motionPool()[6].setTargetPos(angle6);
    controller()->motionPool()[7].setTargetPos(angle7);
    controller()->motionPool()[8].setTargetPos(angle8);
    controller()->motionPool()[9].setTargetPos(angle9);

    if (ret == 0)std::cout << count() << std::endl;
    return ret;
}
auto WalkForward::collectNrt()->void {}
WalkForward::WalkForward(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"walk_step\">"
        "<GroupParam>"
        "       <Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "		<Param name=\"speed\" default=\"0.3\" abbreviation=\"v\"/>"
        "		<Param name=\"high\" default=\"100\" abbreviation=\"h\"/>"
        "		<Param name=\"long\" default=\"100\" abbreviation=\"l\"/>"
        "</GroupParam>"
        "</Command>");
}




////建模
//auto BipedModel::prepareNrt()->void
//{
//    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
//}
//auto BipedModel::executeRT()->int
//{
//    //if (count() == 1)this->master()->logFileRawName("walkStep");
//    static double begin_angle[10]{ 0,0,0,0,0,0,0,0,0,0 };
//
//
//
//    int ret = 1;
//
//    TCurve s1(0.3, 3);
//    s1.getCurveParam();
//    EllipseTrajectory e1(0, 100, 0, s1);
//
//    //步态规划
//    ret = forwardBipedRobot(5, count() - 1, &e1, input_angle);
//
//    
//
//    model()->setInputPos(input_angle);
//    //if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
//
//    model()->setTime(0.001 * count());
//
//    if (ret == 0)std::cout << count() << "\t"<< std::endl;
//
//    return ret;
//}
//BipedModel::BipedModel(const std::string& name)
//{
//    aris::core::fromXmlString(command(),
//        "<Command name=\"biped_model\"/>");
//
//}
//BipedModel::~BipedModel() = default;
//
//
//class Biped :public aris::dynamic::Model {
//
//public:
//    auto virtual init()->void override
//    {
//        this->Model::init();
//
//
//        // 设置身体固定时的拓扑结构 —— 静力学 // 
//        for (auto& m : this->motionPool())m.activate(true);
//        for (auto& gm : this->generalMotionPool())gm.activate(false);
//        for (auto& f : this->forcePool())f.activate(false);
//        this->generalMotionPool()[0].activate(true);
//        this->solverPool()[4].allocateMemory();
//
//        // 站立时的拓扑结构 //
//        for (auto& m : this->motionPool())m.activate(true);
//        for (auto& gm : this->generalMotionPool())gm.activate(true);
//        for (auto& f : this->forcePool())f.activate(false);
//        this->generalMotionPool()[0].activate(false);
//        this->solverPool()[5].allocateMemory();
//
//        // 其他尝试
//        for (auto& m : this->motionPool())m.activate(true);
//        for (auto& gm : this->generalMotionPool())gm.activate(false);
//        for (auto& f : this->forcePool())f.activate(false);
//        this->generalMotionPool()[0].activate(false);
//        this->solverPool()[6].allocateMemory();
//
//    }
//
//    Biped()
//    {
//        ////2.0
//        //// set gravity //
//        //const double gravity[6]{ 0.0,-9.8,0.0,0.0,0.0,0.0 };
//
//        //this->environment().setGravity(gravity);
//
//        ////define joint pos //
//        //const double leg_pe[20][3]{
//        //    { -0.112,       0.0,         -0.2025 },//0 l_A
//        //    { -0.112,      -0.1545,      -0.2025 },//1 l_B
//        //    { -0.112,      -0.2295,      -0.2025 },//2 l_C
//        //    { -0.112,      -0.3045,      -0.2025 },//3 l_D
//        //    { -0.21255,    -0.71437,     -0.2025 },//4 l_E
//        //    { -0.38514,    -0.67985,     -0.2025 },//5 l_F
//        //    { -0.16535,    -0.80253,     -0.2025 },//6 l_G
//        //    { -0.3275,     -0.110123,    -0.2025 },//7 l_H
//        //    { -0.11775,    -0.114123,    -0.2025 },//8 l_I
//        //    { -0.21336,    -0.84636,     -0.2025 },//9 l_J
//        //    { -0.112,       0.0,          0.2025 },//0 r_A
//        //    { -0.112,      -0.1545,       0.2025 },//1 r_B
//        //    { -0.112,      -0.2295,       0.2025 },//2 r_C
//        //    { -0.112,      -0.3045,       0.2025 },//3 r_D
//        //    { -0.21255,    -0.71437,      0.2025 },//4 r_E
//        //    { -0.38514,    -0.67985,      0.2025 },//5 r_F
//        //    { -0.16535,    -0.80253,      0.2025 },//6 r_G
//        //    { -0.3275,     -0.110123,     0.2025 },//7 r_H
//        //    { -0.11775,    -0.114123,     0.2025 },//8 r_I
//        //    { -0.21336,    -0.84636,      0.2025 },//9 r_J
//        //};
//        ////define ee pos //
//        //const double ee_pos[2][6]{
//        //    { -0.333,     -1.14123,    -0.2025,   0.0,  0.0,  0.0 },   //left leg
//        //    { -0.333,     -1.14123,     0.2025,   0.0,  0.0,  0.0 },   //right leg
//        //};
//
//
//        ////iv:  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
//        //// define iv param //  材料都为铝 总重30.86kg  第2.5代
//        //const double body_iv[10]{ 15.1007177439,0,0,0,0.68976308,0.612989762,0.1389151407,0,0,0 };
//        ////leg1
//        //const double lf_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
//        //const double lf_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
//        //const double lf_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
//        ////leg2
//        //const double lr_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
//        //const double lr_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
//        //const double lr_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
//
//
//
//
//        //// add part //
//        //auto& body = this->partPool().add<aris::dynamic::Part>("BODY", body_iv);
//        ////leg1
//        //auto& l_p1_A = this->partPool().add<aris::dynamic::Part>("L_P1_A", lr_p1_iv);
//        //auto& l_p1_B = this->partPool().add<aris::dynamic::Part>("L_P1_B", lr_p1_iv);
//        //auto& l_p2 = this->partPool().add<aris::dynamic::Part>("L_P2", lr_p2_iv);
//        //auto& l_p3 = this->partPool().add<aris::dynamic::Part>("L_P3", lr_p3_iv);
//        //auto& l_p4 = this->partPool().add<aris::dynamic::Part>("L_P4", lr_p1_iv);
//        //auto& l_p5 = this->partPool().add<aris::dynamic::Part>("L_P5", lr_p2_iv);
//        //auto& l_p6 = this->partPool().add<aris::dynamic::Part>("L_P6", lr_p3_iv);
//        //auto& l_p7 = this->partPool().add<aris::dynamic::Part>("L_P7", lr_p1_iv);
//        //auto& l_p8 = this->partPool().add<aris::dynamic::Part>("L_P8", lr_p2_iv);
//
//        ////leg2
//        //auto& r_p1_A = this->partPool().add<aris::dynamic::Part>("R_P1_A", lf_p1_iv);
//        //auto& r_p1_B = this->partPool().add<aris::dynamic::Part>("R_P1_B", lf_p1_iv);
//        //auto& r_p2 = this->partPool().add<aris::dynamic::Part>("R_P2", lf_p2_iv);
//        //auto& r_p3 = this->partPool().add<aris::dynamic::Part>("R_P3", lf_p3_iv);
//        //auto& r_p4 = this->partPool().add<aris::dynamic::Part>("R_P4", lf_p3_iv);
//        //auto& r_p5 = this->partPool().add<aris::dynamic::Part>("R_P5", lf_p3_iv);
//        //auto& r_p6 = this->partPool().add<aris::dynamic::Part>("R_P6", lf_p3_iv);
//        //auto& r_p7 = this->partPool().add<aris::dynamic::Part>("R_P7", lf_p3_iv);
//        //auto& r_p8 = this->partPool().add<aris::dynamic::Part>("R_P8", lf_p3_iv);
//
//
//        //// add geometry //
//        //this->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\ground.x_t");
//        //body.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\Body.x_t");
//        ////leg1
//        //r_p1_A.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_1A.x_t");
//        //r_p1_B.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_1B.x_t");
//        //r_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_2.x_t");
//        //r_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_3.x_t");
//        //r_p4.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_4.x_t");
//        //r_p5.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_5.x_t");
//        //r_p6.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_6.x_t");
//        //r_p7.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_7.x_t");
//        //r_p8.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\r_8.x_t");
//
//        ////leg2
//        //l_p1_A.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_1A.x_t");
//        //l_p1_B.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_1B.x_t");
//        //l_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_2.x_t");
//        //l_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_3.x_t");
//        //l_p4.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_4.x_t");
//        //l_p5.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_5.x_t");
//        //l_p6.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_6.x_t");
//        //l_p7.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_7.x_t");
//        //l_p8.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part\\l_8.x_t");
//
//
//
//
//        //// add joints //
//        ////leg1
//        //auto& l_r1_x = this->addRevoluteJoint(l_p1_A, body, leg_pe[0], std::array<double, 3>{1, 0, 0}.data());//A
//        //auto& l_r1_y = this->addRevoluteJoint(l_p1_B, l_p1_A, leg_pe[0], std::array<double, 3>{0, 1, 0}.data());//A
//        //auto& l_r2 = this->addRevoluteJoint(l_p2, l_p1_B, leg_pe[1], std::array<double, 3>{0, 0, 1}.data());//B
//        //auto& l_r3 = this->addRevoluteJoint(l_p3, l_p2, leg_pe[2], std::array<double, 3>{0, 0, 1}.data());//C
//        //auto& l_r4 = this->addRevoluteJoint(l_p4, l_p2, leg_pe[3], std::array<double, 3>{0, 0, 1}.data());//D
//        //auto& l_r5 = this->addRevoluteJoint(l_p5, l_p4, leg_pe[4], std::array<double, 3>{0, 0, 1}.data());//E
//        //auto& l_r6 = this->addRevoluteJoint(l_p5, l_p3, leg_pe[5], std::array<double, 3>{0, 0, 1}.data());//F
//        //auto& l_r7 = this->addRevoluteJoint(l_p6, l_p5, leg_pe[6], std::array<double, 3>{0, 0, 1}.data());//G
//        //auto& l_r8 = this->addRevoluteJoint(l_p7, l_p5, leg_pe[7], std::array<double, 3>{0, 0, 1}.data());//H
//        //auto& l_r9 = this->addRevoluteJoint(l_p7, l_p8, leg_pe[8], std::array<double, 3>{0, 0, 1}.data());//I
//        //auto& l_r10 = this->addRevoluteJoint(l_p8, l_p6, leg_pe[9], std::array<double, 3>{0, 0, 1}.data());//J
//
//        ////leg2
//        //auto& r_r1_x = this->addRevoluteJoint(r_p1_A, body, leg_pe[10], std::array<double, 3>{1, 0, 0}.data());//A
//        //auto& r_r1_y = this->addRevoluteJoint(r_p1_B, r_p1_A, leg_pe[10], std::array<double, 3>{0, 1, 0}.data());//A
//        //auto& r_r2 = this->addRevoluteJoint(r_p2, r_p1_B, leg_pe[11], std::array<double, 3>{0, 0, 1}.data());//B
//        //auto& r_r3 = this->addRevoluteJoint(r_p3, r_p2, leg_pe[12], std::array<double, 3>{0, 0, 1}.data());//C
//        //auto& r_r4 = this->addRevoluteJoint(r_p4, r_p2, leg_pe[13], std::array<double, 3>{0, 0, 1}.data());//D
//        //auto& r_r5 = this->addRevoluteJoint(r_p5, r_p4, leg_pe[14], std::array<double, 3>{0, 0, 1}.data());//E
//        //auto& r_r6 = this->addRevoluteJoint(r_p5, r_p3, leg_pe[15], std::array<double, 3>{0, 0, 1}.data());//F
//        //auto& r_r7 = this->addRevoluteJoint(r_p6, r_p5, leg_pe[16], std::array<double, 3>{0, 0, 1}.data());//G
//        //auto& r_r8 = this->addRevoluteJoint(r_p7, r_p5, leg_pe[17], std::array<double, 3>{0, 0, 1}.data());//H
//        //auto& r_r9 = this->addRevoluteJoint(r_p7, r_p8, leg_pe[18], std::array<double, 3>{0, 0, 1}.data());//I
//        //auto& r_r10 = this->addRevoluteJoint(r_p8, r_p6, leg_pe[19], std::array<double, 3>{0, 0, 1}.data());//J
//
//
//
//        //// add motion //
//        ////leg1
//        //auto& l_m1 = this->addMotion(l_r1_x);
//        //auto& l_m2 = this->addMotion(l_r1_y);
//        //auto& l_m3 = this->addMotion(l_r2);
//        //auto& l_m4 = this->addMotion(l_r4);
//        //auto& l_m5 = this->addMotion(l_r7);
//        ////leg2
//        //auto& r_m1 = this->addMotion(r_r1_x);
//        //auto& r_m2 = this->addMotion(r_r1_y);
//        //auto& r_m3 = this->addMotion(r_r2);
//        //auto& r_m4 = this->addMotion(r_r4);
//        //auto& r_m5 = this->addMotion(r_r7);
//
//
//        //// add end-effector //
//        //auto body_ee_maki = body.addMarker("body_ee_mak_i");
//        //auto body_ee_makj = this->ground().addMarker("body_ee_mak_j");
//
//        //auto& body_ee = this->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
//        //auto& l_ee = this->addPointMotion(l_p7, this->ground(), ee_pos[0]);
//        //this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
//        //auto& r_ee = this->addPointMotion(r_p7, this->ground(), ee_pos[1]);
//        //this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
//
//
//        //// add force //
//        //auto& f1 = this->forcePool().add<aris::dynamic::GeneralForce>("f1", l_ee.makI(), l_ee.makJ());
//        //auto& f2 = this->forcePool().add<aris::dynamic::GeneralForce>("f2", r_ee.makI(), r_ee.makJ());
//
//
//
//        //auto& inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
//        //auto& forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
//        //auto& inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
//        //auto& forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();
//
//        //auto& fix_body_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
//        //auto& stand_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
//
//        //auto& other = this->solverPool().add<aris::dynamic::UniversalSolver>();
//
//        //// 添加仿真器和仿真结果 //
//        //auto& adams = this->simulatorPool().add<aris::dynamic::AdamsSimulator>();
//        //auto& result = this->simResultPool().add<aris::dynamic::SimResult>();
//
//        //this->init();
//
//
//        // set gravity //
//        const double gravity[6]{ 0.0,-9.8,0.0,0.0,0.0,0.0 };
//
//        this->environment().setGravity(gravity);
//
//        //define joint pos //
//        const double leg_pe[18][3]{
//            { -0.112,       0.0,         -0.2025 },//0 l_A
//            { -0.112,      -0.1545,      -0.2025 },//1 l_B
//            { -0.0414,     -0.28685,     -0.2025 },//2 l_C
//            { -0.22024,    -0.6691,      -0.2025 },//3 l_D
//            { -0.39374,    -0.63953,     -0.2025 },//4 l_E
//            { -0.17054,    -0.75588,     -0.2025 },//5 l_F
//            { -0.02945,    -1.05067,     -0.2025 },//6 l_G
//            { -0.11445,    -1.09067,     -0.2025 },//7 l_H
//            { -0.21924,    -0.79893,     -0.2025 },//8 l_I
//            { -0.112,       0.0,          0.2025 },//9 r_A
//            { -0.112,      -0.1545,       0.2025 },//10 r_B
//            { -0.0414,     -0.28685,      0.2025 },//11 r_C
//            { -0.22024,    -0.6691,       0.2025 },//12 r_D
//            { -0.39374,    -0.63953,      0.2025 },//13 r_E
//            { -0.17054,    -0.75588,      0.2025 },//14 r_F
//            { -0.02945,    -1.05067,      0.2025 },//15 r_G
//            { -0.11445,    -1.09067,      0.2025 },//16 r_H
//            { -0.21924,    -0.79893,      0.2025 },//17 r_I
//        };
//        //define ee pos //
//        const double ee_pos[2][6]{
//            { -0.03,     -1.09068,    -0.2025,   0.0,  0.0,  0.0 },   //left leg
//            { -0.03,     -1.09068,     0.2025,   0.0,  0.0,  0.0 },   //right leg
//        };
//
//
//        //iv:  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
//        // define iv param //  
//        const double body_iv[10]{ 11.31,0,0,0,0.37847841,0.37064928,5.809695E-02,2.8643E-03,1.3766E-04,3.4037E-04 };
//        //leg1
//        const double l_p1_A_iv[10]{ 4.5,0,0,0,1.05123E-02,8.87847E-03,1.093129E-02,2.6628E-04,7.0E-08,-7.2E-07 };
//        const double l_p1_B_iv[10]{ 4.68,0,0,0,1.251268E-02,1.231649E-02,1.035636E-02,0,0,0 };
//        const double l_p2_iv[10]{ 5.78,0,0,0,3.159352E-02,2.057131E-02,2.888912E-02,-5.10262E-03,2.76859E-03,-7.05325E-03 };
//        const double l_p3_iv[10]{ 6.0E-02,0,0,0,1.6783E-03,2.59E-04,1.86625E-03,6.0447E-04,8.834E-05,2.3943E-04 };
//        const double l_p4_iv[10]{ 2.45,0,0,0,4.878217E-02,1.962284E-02,6.539863E-02,2.393965E-02,6.708E-04,2.63216E-03 };
//        const double l_p5_iv[10]{ 2.12,0,0,0,4.953662E-02,8.915E-03,5.596285E-02,-1.668636E-02,7.074E-05,-3.9266E-04 };
//        const double l_p6_iv[10]{ 0.12,0,0,0,1.5811E-04,1.5752E-04,3.0787E-04,1.2455E-04,1.066E-05,1.069E-05 };
//        const double l_p7_iv[10]{ 1.33,0,0,0,9.4366E-04,8.59617E-03,8.33784E-03,-5.3806E-04,-1.2184E-04,3.703E-05 };
//        const double l_p8_iv[10]{ 4.0E-02,0,0,0,4.355E-04,3.459E-05,4.6941E-04,-1.2054E-04,0,0 };
//        //leg2
//        const double r_p1_A_iv[10]{ 4.5,0,0,0,1.05123E-02,8.87847E-03,1.093129E-02,2.6628E-04,7.0E-08,-7.2E-07 };
//        const double r_p1_B_iv[10]{ 4.68,0,0,0,1.251268E-02,1.231649E-02,1.035636E-02,0,0,0 };
//        const double r_p2_iv[10]{ 5.78,0,0,0,3.159352E-02,2.057131E-02,2.888912E-02,-5.10262E-03,-2.76859E-03,7.05325E-03 };
//        const double r_p3_iv[10]{ 6.0E-02,0,0,0,1.6783E-03,2.59E-04,1.86625E-03,6.0447E-04,-8.834E-05,-2.3943E-04 };
//        const double r_p4_iv[10]{ 2.45,0,0,0,4.878217E-02,1.962284E-02,6.539863E-02,2.393965E-02,-6.708E-04,-2.63216E-03 };
//        const double r_p5_iv[10]{ 2.12,0,0,0,4.953662E-02,8.915E-03,5.596285E-02,-1.668636E-02,-7.074E-05,3.9266E-04 };
//        const double r_p6_iv[10]{ 0.12,0,0,0,1.5811E-04,1.5752E-04,3.0787E-04,1.2455E-04,-1.066E-05,-1.069E-05 };
//        const double r_p7_iv[10]{ 1.33,0,0,0,9.4366E-04,8.59617E-03,8.33784E-03,-5.3806E-04,1.2184E-04,-3.703E-05 };
//        const double r_p8_iv[10]{ 4.0E-02,0,0,0,4.355E-04,3.459E-05,4.6941E-04,-1.2054E-04,0,0 };
//
//
//
//
//
//        // add part //
//        auto& body = this->partPool().add<aris::dynamic::Part>("BODY", body_iv);
//        //leg1
//        auto& l_p1_A = this->partPool().add<aris::dynamic::Part>("L_P1_A", l_p1_A_iv);
//        auto& l_p1_B = this->partPool().add<aris::dynamic::Part>("L_P1_B", l_p1_B_iv);
//        auto& l_p2 = this->partPool().add<aris::dynamic::Part>("L_P2", l_p2_iv);
//        auto& l_p3 = this->partPool().add<aris::dynamic::Part>("L_P3", l_p3_iv);
//        auto& l_p4 = this->partPool().add<aris::dynamic::Part>("L_P4", l_p4_iv);
//        auto& l_p5 = this->partPool().add<aris::dynamic::Part>("L_P5", l_p5_iv);
//        auto& l_p6 = this->partPool().add<aris::dynamic::Part>("L_P6", l_p6_iv);
//        auto& l_p7 = this->partPool().add<aris::dynamic::Part>("L_P7", l_p7_iv);
//        auto& l_p8 = this->partPool().add<aris::dynamic::Part>("L_P8", l_p8_iv);
//
//        //leg2
//        auto& r_p1_A = this->partPool().add<aris::dynamic::Part>("R_P1_A", r_p1_A_iv);
//        auto& r_p1_B = this->partPool().add<aris::dynamic::Part>("R_P1_B", r_p1_B_iv);
//        auto& r_p2 = this->partPool().add<aris::dynamic::Part>("R_P2", r_p2_iv);
//        auto& r_p3 = this->partPool().add<aris::dynamic::Part>("R_P3", r_p3_iv);
//        auto& r_p4 = this->partPool().add<aris::dynamic::Part>("R_P4", r_p4_iv);
//        auto& r_p5 = this->partPool().add<aris::dynamic::Part>("R_P5", r_p5_iv);
//        auto& r_p6 = this->partPool().add<aris::dynamic::Part>("R_P6", r_p6_iv);
//        auto& r_p7 = this->partPool().add<aris::dynamic::Part>("R_P7", r_p7_iv);
//        auto& r_p8 = this->partPool().add<aris::dynamic::Part>("R_P8", r_p8_iv);
//
//
//        // add geometry //
//        this->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\ground.x_t");
//        body.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\Body.x_t");
//        //leg1
//        r_p1_A.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p1_A.x_t");
//        r_p1_B.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p1_B.x_t");
//        r_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p2.x_t");
//        r_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p3.x_t");
//        r_p4.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p4.x_t");
//        r_p5.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p5.x_t");
//        r_p6.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p6.x_t");
//        r_p7.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p7.x_t");
//        r_p8.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\r_p8.x_t");
//
//        //leg2
//        l_p1_A.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p1_A.x_t");
//        l_p1_B.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p1_B.x_t");
//        l_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p2.x_t");
//        l_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p3.x_t");
//        l_p4.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p4.x_t");
//        l_p5.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p5.x_t");
//        l_p6.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p6.x_t");
//        l_p7.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p7.x_t");
//        l_p8.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\54067\\Desktop\\Adams-final\\model\\part1.1\\l_p8.x_t");
//
//
//
//
//        // add joints //
//        //leg1
//        auto& l_r1_x = this->addRevoluteJoint(l_p1_A, body, leg_pe[0], std::array<double, 3>{1, 0, 0}.data());//A
//        auto& l_r1_y = this->addRevoluteJoint(l_p1_B, l_p1_A, leg_pe[0], std::array<double, 3>{0, 1, 0}.data());//A
//        auto& l_r2 = this->addRevoluteJoint(l_p2, l_p1_B, leg_pe[1], std::array<double, 3>{0, 0, 1}.data());//B
//        auto& l_r3 = this->addRevoluteJoint(l_p3, l_p2, leg_pe[1], std::array<double, 3>{0, 0, 1}.data());//B
//        auto& l_r4 = this->addRevoluteJoint(l_p4, l_p2, leg_pe[2], std::array<double, 3>{0, 0, 1}.data());//C
//        auto& l_r5 = this->addRevoluteJoint(l_p5, l_p4, leg_pe[3], std::array<double, 3>{0, 0, 1}.data());//E
//        auto& l_r6 = this->addRevoluteJoint(l_p5, l_p3, leg_pe[4], std::array<double, 3>{0, 0, 1}.data());//F
//        auto& l_r7 = this->addRevoluteJoint(l_p6, l_p5, leg_pe[5], std::array<double, 3>{0, 0, 1}.data());//G
//        auto& l_r8 = this->addRevoluteJoint(l_p7, l_p5, leg_pe[6], std::array<double, 3>{0, 0, 1}.data());//H
//        auto& l_r9 = this->addRevoluteJoint(l_p7, l_p8, leg_pe[7], std::array<double, 3>{0, 0, 1}.data());//I
//        auto& l_r10 = this->addRevoluteJoint(l_p8, l_p6, leg_pe[8], std::array<double, 3>{0, 0, 1}.data());//J
//
//        //leg2
//        auto& r_r1_x = this->addRevoluteJoint(r_p1_A, body, leg_pe[9], std::array<double, 3>{1, 0, 0}.data());//A
//        auto& r_r1_y = this->addRevoluteJoint(r_p1_B, r_p1_A, leg_pe[9], std::array<double, 3>{0, 1, 0}.data());//A
//        auto& r_r2 = this->addRevoluteJoint(r_p2, r_p1_B, leg_pe[10], std::array<double, 3>{0, 0, 1}.data());//B
//        auto& r_r3 = this->addRevoluteJoint(r_p3, r_p2, leg_pe[10], std::array<double, 3>{0, 0, 1}.data());//C
//        auto& r_r4 = this->addRevoluteJoint(r_p4, r_p2, leg_pe[11], std::array<double, 3>{0, 0, 1}.data());//D
//        auto& r_r5 = this->addRevoluteJoint(r_p5, r_p4, leg_pe[12], std::array<double, 3>{0, 0, 1}.data());//E
//        auto& r_r6 = this->addRevoluteJoint(r_p5, r_p3, leg_pe[13], std::array<double, 3>{0, 0, 1}.data());//F
//        auto& r_r7 = this->addRevoluteJoint(r_p6, r_p5, leg_pe[14], std::array<double, 3>{0, 0, 1}.data());//G
//        auto& r_r8 = this->addRevoluteJoint(r_p7, r_p5, leg_pe[15], std::array<double, 3>{0, 0, 1}.data());//H
//        auto& r_r9 = this->addRevoluteJoint(r_p7, r_p8, leg_pe[16], std::array<double, 3>{0, 0, 1}.data());//I
//        auto& r_r10 = this->addRevoluteJoint(r_p8, r_p6, leg_pe[17], std::array<double, 3>{0, 0, 1}.data());//J
//
//
//
//        // add motion //
//        //leg1
//        auto& l_m1 = this->addMotion(l_r1_x);
//        auto& l_m2 = this->addMotion(l_r1_y);
//        auto& l_m3 = this->addMotion(l_r2);
//        auto& l_m4 = this->addMotion(l_r4);
//        auto& l_m5 = this->addMotion(l_r7);
//        //leg2
//        auto& r_m1 = this->addMotion(r_r1_x);
//        auto& r_m2 = this->addMotion(r_r1_y);
//        auto& r_m3 = this->addMotion(r_r2);
//        auto& r_m4 = this->addMotion(r_r4);
//        auto& r_m5 = this->addMotion(r_r7);
//
//        double ee_pm_l[16] =
//        {
//            1,0,0,ee_pos[0][0],
//            0,1,0,ee_pos[0][1],
//            0,0,1,ee_pos[0][2],
//            0,0,0,1
//        };
//
//        double ee_pm_r[16] =
//        {
//            1,0,0,ee_pos[1][0],
//            0,1,0,ee_pos[1][1],
//            0,0,1,ee_pos[1][2],
//            0,0,0,1
//        };
//
//        // add end-effector //
//        auto body_ee_maki = body.addMarker("body_ee_mak_i");
//        auto body_ee_makj = this->ground().addMarker("body_ee_mak_j");
//
//        auto& body_ee = this->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
//        auto& l_ee = this->addPointMotion(l_p7, this->ground(), ee_pos[0]);
//        this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
//        //this->ground().markerPool().back().setPrtPm(ee_pm_l);
//        auto& r_ee = this->addPointMotion(r_p7, this->ground(), ee_pos[1]);
//        this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
//        //this->ground().markerPool().back().setPrtPm(ee_pm_r);
//
//        // add force //
//        auto& f1 = this->forcePool().add<aris::dynamic::GeneralForce>("f1", l_ee.makI(), l_ee.makJ());
//        auto& f2 = this->forcePool().add<aris::dynamic::GeneralForce>("f2", r_ee.makI(), r_ee.makJ());
//
//
//
//        auto& inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
//        auto& forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
//        auto& inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
//        auto& forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();
//
//        auto& fix_body_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
//        auto& stand_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
//
//        auto& other = this->solverPool().add<aris::dynamic::UniversalSolver>();
//
//        // 添加仿真器和仿真结果 //
//        auto& adams = this->simulatorPool().add<aris::dynamic::AdamsSimulator>();
//        auto& result = this->simResultPool().add<aris::dynamic::SimResult>();
//
//        this->init();
//    }
//
//};
//
//auto createModelBiped()->std::unique_ptr<aris::dynamic::Model>
//{
//    return std::unique_ptr<aris::dynamic::Model>(new Biped);
//}


auto createControllerBiped()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 10; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION

        std::cout << "using simulation" << std::endl;

        double pos_offset[10]
        {
            0,0,0,0,0,
            0,0,0,0,0,
        };
#else
        std::cout << "not using simulation" << std::endl;

        //调试
        double pos_offset[10]
        {
             0,
             0,
             0,
             0,
             0,
             0,
             0,
             0,
             0,
             0
        };

        //实际用
        //double pos_offset[10]
        //{
        //     0.644464,
        //     0,
        //     0,
        //     0,
        //     0,
        //     0,
        //     0,
        //     0,
        //     0,
        //     2.58562,
        //    //  0.389823,
        //    //-0.205458,
        //    //-0.541879,
        //    //0.904521,
        //    //0.785734,
        //    //-0.136476,
        //    //-2.81231,
        //    //2.60734,
        //    //0.0327409,
        //    //-0.425296,
        //};


#endif
        double pos_factor[10]
        {
             131072.0 / 2 / PI,  131072.0 / 2 / PI,  -131072.0 / 2 / PI,  -131072.0 / 2 / PI, -131072.0 / 2 / PI,
            -131072.0 / 2 / PI, -131072.0 / 2 / PI,   131072.0 / 2 / PI,  -131072.0 / 2 / PI, -131072.0 / 2 / PI
        };

        // 调试用
        double max_pos[10]
        {
            PI + 1000, PI + 1000, PI + 1000, PI + 1000, PI + 1000,
            PI + 1000, PI + 1000, PI + 1000, PI + 1000, PI + 1000,
        };
        double min_pos[10]
        {
            -PI - 1000, -PI - 1000, -PI - 1000, -PI - 1000, -PI - 1000,
            -PI - 1000, -PI - 1000, -PI - 1000, -PI - 1000, -PI - 1000,
        };

        ////实际用
        //double max_pos[10]
        //{
        //    PI + 1000,  0,       PI / 2,   PI / 2,  PI / 4,
        //    PI / 4,     PI / 2,  PI / 2,   0,       PI + 1000,
        //};
        //double min_pos[10]
        //{
        //    -PI - 1000, -PI * 3 / 4,  0,  -PI / 2,     -PI / 4,
        //    -PI / 4,    -PI / 2,      0,  -PI * 3 / 4, -PI - 1000,
        //};

        double max_vel[10]
        {
            3300.0 / 60 * 2 * PI, 3300.0 / 60 * 2 * PI, 3300.0 / 60 * 2 * PI, 3300.0 / 60 * 2 * PI, 3300.0 / 60 * 2 * PI, 
            3300.0 / 60 * 2 * PI, 3300.0 / 60 * 2 * PI, 3300.0 / 60 * 2 * PI, 3300.0 / 60 * 2 * PI, 3300.0 / 60 * 2 * PI
        };
        double max_acc[10]
        {
            30000.0, 30000.0, 30000.0, 30000.0, 30000.0,
            30000.0, 30000.0, 30000.0, 30000.0, 30000.0
        };

        int phy_id[10]={0,1,2,3,4,5,6,7,8,9};


        std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
        "	<SyncManagerPoolObject>"
        "		<SyncManager is_tx=\"false\"/>"
        "		<SyncManager is_tx=\"true\"/>"
        "		<SyncManager is_tx=\"false\">"
        "			<Pdo index=\"0x1600\" is_tx=\"false\">"
        "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
        "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"offset_vel\" index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"tor_offset\" index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
        "			</Pdo>"
        "		</SyncManager>"
        "		<SyncManager is_tx=\"true\">"
        "			<Pdo index=\"0x1a00\" is_tx=\"true\">"
        "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
        "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"following_error\" index=\"0x60F4\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
        "				<PdoEntry name=\"cur_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
        "			</Pdo>"
        "		</SyncManager>"
        "	</SyncManagerPoolObject>"
            "</EthercatMotor>";

        aris::core::fromXmlString(controller->slavePool().add<aris::control::EthercatMotor>(), xml_str);


#ifdef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setVirtual(true);
#endif

#ifndef WIN32
        std::cout << "before" << std::endl;
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
        //dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).writeSdo(xml_str);

        std::cout << "end" << std::endl;
#endif

    };
    return controller;
}
auto createPlanBiped()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();

    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();



    //自己写的命令
    //plan_root->planPool().add<Enable2>();
     //plan_root->planPool().add<MoveJoint>();
     plan_root->planPool().add<ReadPosition>();
     plan_root->planPool().add<ReadCurrent>();
     plan_root->planPool().add<WalkStep>();
     //plan_root->planPool().add<testIK>();
     plan_root->planPool().add<TestMotor>();
     plan_root->planPool().add<PositionCheck>();
     plan_root->planPool().add<CheckEnable>();
     plan_root->planPool().add<RobotPrepare>();
     plan_root->planPool().add<MoveEnd>();
     //plan_root->planPool().add<BipedModel>();
     plan_root->planPool().add<WalkForward>();
     

    return plan_root;
}

}
