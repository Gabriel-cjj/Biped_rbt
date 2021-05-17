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
    0,0,0,0,0,
    0,0,0,0,0
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

auto MoveJoint::prepareNrt()->void
{
    dir_ = doubleParam("direction");

    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto MoveJoint::executeRT()->int
{
    static double begin_angle[2];

       if (count() == 1)
       {
           begin_angle[0] = controller()->motionPool()[0].targetPos();
       }

       TCurve s1(0.016,0.3);
       s1.getCurveParam();
       double angle0 = begin_angle[0] + dir_  *  s1.getTCurve(count());


       controller()->motionPool()[0].setTargetPos(angle0);
       return s1.getTc() * 1000-count();
}
auto MoveJoint::collectNrt()->void {}
MoveJoint::MoveJoint(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"test_mvj\">"
        "		<Param name=\"direction\" default=\"1.0\" abbreviation=\"d\"/>"
        "</Command>");
}

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
        controller()->motionPool()[5].setPosOffset(-2 * PI);
    }

    if (begin_angle[7] < 0)
    {
        begin_angle[7] = begin_angle[7] + 2 * PI;
        controller()->motionPool()[6].setPosOffset(+2 * PI);
    }

    if (begin_angle[9] < 0)
    {
        begin_angle[9] = begin_angle[9] + 2 * PI;
        controller()->motionPool()[6].setPosOffset(+2 * PI);
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
    if (count() == 1)this->master()->logFileRawName("actul_current");

    if (count() == 1)ret = ret - 1;

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
    static double begin_angle[10]{0,0,0,0,0,0,0,0,0,0};
    int ret = 1;

    if (count() == 1)
    {
        //begin_angle[0] = controller()->motionPool()[0].actualPos();
        //begin_angle[1] = controller()->motionPool()[1].actualPos();
        //begin_angle[2] = controller()->motionPool()[2].actualPos();
        //begin_angle[3] = controller()->motionPool()[3].actualPos();
        //begin_angle[4] = controller()->motionPool()[4].actualPos();
        //begin_angle[5] = controller()->motionPool()[5].actualPos();
        //begin_angle[6] = controller()->motionPool()[6].actualPos();
        //begin_angle[7] = controller()->motionPool()[7].actualPos();
        //begin_angle[8] = controller()->motionPool()[8].actualPos();
        //begin_angle[9] = controller()->motionPool()[9].actualPos();
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

        //输出身体和足尖曲线
        for (int j = 0; j < 6; j++)
        {
            lout() << file_current_leg[j] << "\t";

            //if (ret < 31)
            //{
            //    //if (ret < 60)
            //    //{

            //        mout() << file_current_leg[j] << "  ";
            //    //}
            //}
        }
        //if (ret < 31)
        //{
        //    //if (ret < 60)
        //    //{
        //    //    
        //        mout() << time_test << "\n";
        //    //}
        //    
        //}
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;

        //输出电流

        for (int i = 0; i < 10; i++)
        {
            current[i] = this->ecController()->motionPool()[i].readPdo(0x6077, 0x00, current[i]);
            lout() << current[i] << "\t";
        }

    }
   
    double angle0 = begin_angle[0] - input_angle[4];
    double angle1 = begin_angle[1] + input_angle[3];
    double angle2 = begin_angle[2] + input_angle[2];
    double angle3 = begin_angle[3] + input_angle[1];
    double angle4 = begin_angle[4] + input_angle[0];
    double angle5 = begin_angle[5] + input_angle[5];
    double angle6 = begin_angle[6] + input_angle[6];
    double angle7 = begin_angle[7] - input_angle[7];
    double angle8 = begin_angle[8] - input_angle[8];
    double angle9 = begin_angle[9] + input_angle[9];


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
    static double begin_angle[10]{0,0,0,0,0,0,0,0,0,0};
    if (count() == 1)
    {
        //begin_angle[0] = controller()->motionPool()[0].actualPos();
        //begin_angle[1] = controller()->motionPool()[1].actualPos();
        //begin_angle[2] = controller()->motionPool()[2].actualPos();
        //begin_angle[3] = controller()->motionPool()[3].actualPos();
        //begin_angle[4] = controller()->motionPool()[4].actualPos();
        //begin_angle[5] = controller()->motionPool()[5].actualPos();
        //begin_angle[6] = controller()->motionPool()[6].actualPos();
        //begin_angle[7] = controller()->motionPool()[7].actualPos();
        //begin_angle[8] = controller()->motionPool()[8].actualPos();
        //begin_angle[9] = controller()->motionPool()[9].actualPos();

    }

    TCurve s1(0.016, 0.3);
    s1.getCurveParam();



    ikForBipedRobotforTest(x1_, y1_, z1_, 1, 0, 0, l1_, input_angle + 0 * 5);
    ikForBipedRobotforTest(x2_, y2_, z2_, 1, 0, 0, l2_, input_angle + 1 * 5);

    foot_position_start_point[0] = x1_ - kBodyLong;
    foot_position_start_point[1] = y1_;
    foot_position_start_point[2] = z1_ - kBodyWidth / 2;
    foot_position_start_point[3] = x2_ - kBodyLong;
    foot_position_start_point[4] = y2_;
    foot_position_start_point[5] = z2_ + kBodyWidth / 2;

    double angle0 = begin_angle[0] - input_angle[4] * s1.getTCurve(count());
    double angle1 = begin_angle[1] + input_angle[3] * s1.getTCurve(count());
    double angle2 = begin_angle[2] + input_angle[2] * s1.getTCurve(count());
    double angle3 = begin_angle[3] + input_angle[1] * s1.getTCurve(count());
    double angle4 = begin_angle[4] + input_angle[0] * s1.getTCurve(count());
    double angle5 = begin_angle[5] + input_angle[5] * s1.getTCurve(count());
    double angle6 = begin_angle[6] + input_angle[6] * s1.getTCurve(count());
    double angle7 = begin_angle[7] - input_angle[7] * s1.getTCurve(count());
    double angle8 = begin_angle[8] - input_angle[8] * s1.getTCurve(count());
    double angle9 = begin_angle[9] + input_angle[9] * s1.getTCurve(count());


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
        double pos_offset[10]
        {
    0.644464,
    -0.736742,
    -0.515609,
    0.887983,
    0.798197,
    -0.123869,
    -2.88542,
    2.54521,
    -0.0645231,
    2.58562,
        };
#endif
        double pos_factor[10]
        {
            131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI,
            131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI
        };
        double max_pos[10]
        {
            PI + 1000,  PI / 4,  0.3735,   PI / 4,  PI / 4,
            PI / 4,     PI / 2,  1.1937,   2.0269,   PI / 4,
        };
        double min_pos[10]
        {
            -PI - 1000, -2.0269,  -1.1937,   -PI / 2, -PI / 4,
            -PI / 4,     -PI / 4,  -0.3735,  -PI / 4, -PI / 2,
        };
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
     plan_root->planPool().add<MoveJoint>();
     plan_root->planPool().add<ReadPosition>();
     plan_root->planPool().add<ReadCurrent>();
     plan_root->planPool().add<WalkStep>();
     //plan_root->planPool().add<testIK>();
     plan_root->planPool().add<TestMotor>();
     plan_root->planPool().add<PositionCheck>();
     plan_root->planPool().add<CheckEnable>();
     plan_root->planPool().add<RobotPrepare>();
     plan_root->planPool().add<MoveEnd>();


    return plan_root;
}

}
