#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>

#include "robot.h"
#include "plan.h"

#include "kinematics.h"

double input_angle[10] = {0};

////输出参数，模型曲线测试使用
//double file_current_leg[6];
//double file_current_body[16];
//double time_test;

//输出参数，模型曲线测试使用
extern double file_current_leg[6];
extern double file_current_body[16];
extern double time_test;



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
        //begin_angle[9] = controller()->motionPool()[9].actualPos();


        //controller()->motionPool()[0].actualPos();
        //controller()->motionPool()[0].setPosOffset();
        //controller()->motionPool();




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

//auto RobotPrepare::prepareNrt()->void
//{
//    dir_ = doubleParam("direction");
//    motornumber_ = doubleParam("motor_number");
//
//    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
//}
//auto RobotPrepare::executeRT()->int
//{
//    static double begin_angle[10];
//
//    if (count() == 1)
//    {
//        begin_angle[0] = controller()->motionPool()[0].actualPos();
//        begin_angle[1] = controller()->motionPool()[1].actualPos();
//        begin_angle[2] = controller()->motionPool()[2].actualPos();
//        begin_angle[3] = controller()->motionPool()[3].actualPos();
//        begin_angle[4] = controller()->motionPool()[4].actualPos();
//        begin_angle[5] = controller()->motionPool()[5].actualPos();
//        begin_angle[6] = controller()->motionPool()[6].actualPos();
//        begin_angle[7] = controller()->motionPool()[7].actualPos();
//        begin_angle[8] = controller()->motionPool()[8].actualPos();
//        //begin_angle[9] = controller()->motionPool()[9].actualPos();
//    }
//
//    TCurve s1(0.016, 0.3);
//    s1.getCurveParam();
//    double angle0 = begin_angle[motornumber_] + dir_ * s1.getTCurve(count());
//
//
//    controller()->motionPool()[motornumber_].setTargetPos(angle0);
//    return s1.getTc() * 1000 - count();
//}
//auto RobotPrepare::collectNrt()->void {}
//RobotPrepare::RobotPrepare(const std::string& name)
//{
//    aris::core::fromXmlString(command(),
//        "<Command name=\"test_motor\">"
//        "<GroupParam>"
//        "		<Param name=\"direction\" default=\"0.3\" abbreviation=\"d\"/>"
//        "		<Param name=\"motor_number\" default=\"0\" abbreviation=\"m\"/>"
//        "</GroupParam>"
//        "</Command>");
//}

//原地踏步
auto WalkStep::prepareNrt()->void
{
    step_ = doubleParam("step");
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
        //begin_angle[9] = controller()->motionPool()[9].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 100, 0, s1);

   

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
    }

    //发送电机角度
    for (int i = 0; i < 10; i++)
    {
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }



    return ret;


}
auto WalkStep::collectNrt()->void {}
WalkStep::WalkStep(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"walk_step\">"
        "       <Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
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
        //begin_angle[9] = controller()->motionPool()[9].actualPos();
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

//反解测试
auto testIK::prepareNrt()->void
{
    //x_ = doubleParam("x");
    //y_ = doubleParam("y");
    //z_ = doubleParam("z");
    //a_ = doubleParam("a");
    //b_ = doubleParam("b");
    //c_ = doubleParam("c");
    //l_ = doubleParam("l");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto testIK::executeRT()->int
{
    //double end_position[3] =
    //{
    //    65, -1131.9101101572, 0
    //};
    double end_pointing[3] =
    {
        1, 0, 0
    };
    double end_position_on_foot[1] = { 150 };


    //ikForBipedRobotforTest(end_position[0], end_position[1], end_position[2], end_pointing[0], end_pointing[1], end_pointing[2], end_position_on_foot, input_angle);

    double current_leg_in_ground[3] =
    {
        -kBodyLong, 0, -kBodyWidth / 2,
    };
    double current_body_in_ground[3] =
    {
        file_current_body[3], file_current_body[7], file_current_body[11]
    };

    inverseCalculation(current_leg_in_ground, current_body_in_ground, end_pointing, end_position_on_foot, input_angle);
    
    for (int i = 0; i < 6; i++)
    {
        mout() << input_angle[i] << "\t\n";
    }

    return 0;
}
auto testIK::collectNrt()->void {}
testIK::testIK(const std::string& name)
{
    aris::core::fromXmlString(command(),
        //"<Command name=\"test_ik\">"
        //"       <Param name=\"x\" default=\"1\" abbreviation=\"n\"/>"
        //"       <Param name=\"y\" default=\"1\" abbreviation=\"n\"/>"
        //"       <Param name=\"z\" default=\"1\" abbreviation=\"n\"/>"
        //"       <Param name=\"a\" default=\"1\" abbreviation=\"n\"/>"
        //"       <Param name=\"b\" default=\"1\" abbreviation=\"n\"/>"
        //"       <Param name=\"l\" default=\"1\" abbreviation=\"n\"/>"
        //"</Command>");
        "<Command name=\"test_ik\"/>");
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
        //begin_angle[9] = controller()->motionPool()[9].actualPos();
    }

    if (begin_angle[5] > 0)
    {
        begin_angle[5] = begin_angle[5] - 2 * PI;
        controller()->motionPool()[5].setPosOffset(-2 * PI);
    }

    if (begin_angle[6]<0) 
    {
        begin_angle[6] = begin_angle[6] + 2 * PI;
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





//ARIS_REGISTRATION
//{
//    aris::core::class_<MoveJS>("MoveJS").inherit<Plan>();
//}


auto createControllerBiped()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 9; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[10]
        {
            0,0,0,0,0,
            0,0,0,0,0
        };
#else

        double pos_offset[10]
        {
            0,0,0,0,0,
            0,0,0,0,0
        };
#endif
        double pos_factor[10]
        {
            131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI,
            131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI, 131072.0 / 2 / PI
        };
        double max_pos[10]
        {
            PI + 1000.0, PI + 1000.0, 1, 2.1, 1.1,
            0.5, -2.2, 3.8, 0.6, PI + 1000.0,
        };
        double min_pos[10]
        {
            -PI - 1000.0, -PI - 1000.0, -1.6, -0.3, -0.2,
            -0.5,-3.3, 3.2, 0.4, -PI - 1000.0,
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
     plan_root->planPool().add<WalkStep>();
     plan_root->planPool().add<testIK>();
     plan_root->planPool().add<TestMotor>();
     plan_root->planPool().add<PositionCheck>();
     //plan_root->planPool().add<RobotPrepare>();


    return plan_root;
}

}
