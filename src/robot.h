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



class ReadPosition : public aris::core::CloneObject<ReadPosition, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit ReadPosition(const std::string& name = "ReadPosition");

};

class ReadCurrent : public aris::core::CloneObject<ReadCurrent, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit ReadCurrent(const std::string& name = "ReadCurrent");

private:
    int ret;

};


class WalkStep : public aris::core::CloneObject<WalkStep, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit WalkStep(const std::string& name = "WalkStep");
private:
    double step_ = 0;
    double v_ = 0;
    double h_ = 0;

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

//class testIK : public aris::core::CloneObject<testIK, aris::plan::Plan>
//{
//public:
//    auto virtual prepareNrt()->void;
//    auto virtual executeRT()->int;
//    auto virtual collectNrt()->void;
//
//    explicit testIK(const std::string& name = "testIK");
//private:
//    double x_;
//    double y_;
//    double z_;
//    double a_;
//    double b_;
//    double c_;
//    double l_;
//
//};

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

class MoveEnd : public aris::core::CloneObject<MoveEnd, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit MoveEnd(const std::string& name = "MoveEnd");
private:
    double x1_;
    double y1_;
    double z1_;
    double x2_;
    double y2_;
    double z2_;
    double l1_;
    double l2_;

};

class WalkForward : public aris::core::CloneObject<WalkForward, aris::plan::Plan>
{
public:
    auto virtual prepareNrt()->void;
    auto virtual executeRT()->int;
    auto virtual collectNrt()->void;

    explicit WalkForward(const std::string& name = "WalkForward");
private:
    double step_ = 0;
    double v_ = 0;
    double h_ = 0;
    double l_ = 0;

};

//
//// cpp和adams测试 //
//class BipedModel :public aris::core::CloneObject<BipedModel, aris::plan::Plan> {
//public:
//    auto virtual prepareNrt()->void;
//    auto virtual executeRT()->int override;
//
//    virtual ~BipedModel();
//    explicit BipedModel(const std::string& name = "biped_model");
//
//};
//
//
//    auto createModelBiped()->std::unique_ptr<aris::dynamic::Model>;  


    auto createControllerBiped()->std::unique_ptr<aris::control::Controller>;
    auto createPlanBiped()->std::unique_ptr<aris::plan::PlanRoot>;
}



#endif
