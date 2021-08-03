#include <aris.hpp>
#include "robot.h"



int main(int argc, char *argv[])
{
	auto&cs = aris::server::ControlServer::instance();
	
//    aris::control::EthercatController c;
//    c.scan();



    cs.resetController(robot::createControllerBiped().release());
    cs.resetPlanRoot(robot::createPlanBiped().release());

	//cs.resetModel(robot::createModelBiped().release());
	//------------------adams------------------//


	//auto& adams = dynamic_cast<aris::dynamic::AdamsSimulator&>(cs.model().simulatorPool().front());
	//adams.saveAdams("C:\\Users\\54067\\Desktop\\Adams-final\\Biped.cmd");

	//robot::BipedModel plan;
	//adams.simulate(plan, cs.model().simResultPool().front());
	//adams.saveAdams("C:\\Users\\54067\\Desktop\\Adams-final\\Biped.cmd", cs.model().simResultPool().front());


    cs.init();

	//开启WebSocket/socket服务器//
    cs.open(); 

    cs.start();



//    auto &ec = dynamic_cast<aris::control::EthercatController&>(cs.controller());

//    std::uint8_t data[10]{2,0,0,0,0,0,0,0,0,0};


//    ec.slavePool()[0].writeSdo(0x2193, 2, data, 10);
////    ec.slavePool()[0].writeSdo(0x2193, 1, std::uint16_t(0));
////    ec.slavePool()[0].writeSdo(0x2193, 2, std::uint8_t(0));
////    ec.slavePool()[0].writeSdo(0x2193, 3, std::uint8_t(0));
////    ec.slavePool()[0].writeSdo(0x2193, 4, std::uint8_t(0));
////    ec.slavePool()[0].writeSdo(0x2193, 5, std::uint8_t(0));
////    ec.slavePool()[0].writeSdo(0x2193, 6, std::uint8_t(0));
////    ec.slavePool()[0].writeSdo(0x2193, 7, std::uint8_t(0));
////    ec.slavePool()[0].writeSdo(0x2193, 8, std::uint8_t(0));
////    ec.slavePool()[0].writeSdo(0x2193, 9, std::uint8_t(0));

//    ec.slavePool()[0].writeSdo(0x2194, 0, std::uint16_t(0x0000));


//    std::this_thread::sleep_for(std::chrono::seconds(3));
//    std::cout <<"sleep end" << std::endl;

//    ec.slavePool()[0].writeSdo(0x2194, 0, std::uint16_t(0x0002));


//    std::this_thread::sleep_for(std::chrono::seconds(1));
//    std::uint8_t data2[10]{0,0x01,0,0x40,0,0,0,0,0,0};
//    ec.slavePool()[0].writeSdo(0x2193, 2, data2, 10);





//    ec.slavePool()[0].writeSdo(0x2194, 1, std::uint16_t(1));
//    ec.slavePool()[0].writeSdo(0x2194, 2, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2194, 3, std::uint16_t(4));
//    ec.slavePool()[0].writeSdo(0x2194, 4, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2194, 5, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2194, 6, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2194, 7, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2194, 8, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2194, 9, std::uint16_t(0));

//    ec.slavePool()[0].writeSdo(0x2193, 0, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2193, 1, std::uint16_t(1));
//    ec.slavePool()[0].writeSdo(0x2193, 2, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2193, 3, std::uint16_t(4));
//    ec.slavePool()[0].writeSdo(0x2193, 4, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2193, 5, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2193, 6, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2193, 7, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2193, 8, std::uint16_t(0));
//    ec.slavePool()[0].writeSdo(0x2193, 9, std::uint16_t(0));


	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
	cs.runCmdLine();

	return 0;
}
