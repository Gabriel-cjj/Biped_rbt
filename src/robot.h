#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{

using Size = std::size_t;
constexpr double PI = 3.141592653589793;

//class MoveJS : public aris::core::CloneObject<MoveJS, aris::plan::Plan>
//{
//public:
//    auto virtual prepareNrt()->void;
//    auto virtual executeRT()->int;
//    auto virtual collectNrt()->void;
//
//    explicit MoveJS(const std::string &name = "MoveJS_plan");
//
//};

class MoveJoint : public aris::core::CloneObject<MoveJoint, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit MoveJoint(const std::string &name = "MoveJiont");
private:
    double dir_;
};

class ReadPosition : public aris::core::CloneObject<ReadPosition, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit ReadPosition(const std::string& name = "ReadPosition");

};

//class RobotPrepare : public aris::core::CloneObject<RobotPrepare, aris::plan::Plan>
//{
//public:
//    auto virtual prepareNrt()->void;
//    auto virtual executeRT()->int;
//    auto virtual collectNrt()->void;
//
//    explicit RobotPrepare(const std::string& name = "RobotPrepare");
//
//};

class WalkStep : public aris::core::CloneObject<WalkStep, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit WalkStep(const std::string& name = "WalkStep");
private:
    double step_;
    double v_;

};

class TestMotor : public aris::core::CloneObject<TestMotor, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit TestMotor(const std::string& name = "TestMotor");
private:
    double dir_;
    int motornumber_;

};

class testIK : public aris::core::CloneObject<testIK, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit testIK(const std::string& name = "testIK");
private:
    double x_;
    double y_;
    double z_;
    double a_;
    double b_;
    double c_;
    double l_;

};

class PositionCheck : public aris::core::CloneObject<PositionCheck, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit PositionCheck(const std::string& name = "PositionCheck");
private:


};

class CheckEnable : public aris::core::CloneObject<CheckEnable, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit CheckEnable(const std::string& name = "CheckEnable");
private:
    double dir_;

};

class RobotPrepare : public aris::core::CloneObject<RobotPrepare, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit RobotPrepare(const std::string& name = "RobotPrepare");
private:

};



    auto createControllerBiped()->std::unique_ptr<aris::control::Controller>;
    auto createPlanBiped()->std::unique_ptr<aris::plan::PlanRoot>;
}



#endif
