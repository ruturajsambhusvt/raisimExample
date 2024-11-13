#include "raisim/OgreVis.hpp"
// #include "raisimBasicImguiPanel.hpp"
// #include "raisimKeyboardCallback.hpp"
// #include "helper.hpp"
//for Unity
#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
//HDSRL header
//#include "v60_controller/locomotion_controller.h"
// #include "a1_controller/locomotion_planner.h"
#include <math.h>
//For multithreading
#include "pthread.h"

//header helps stop while loop with ctrl+c
#include "signal.h"


int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  // auto laikago = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\laikago\\laikago.urdf");
  // auto A1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\..\\rsc\\A1\\A1_w_kinova.urdf");
  auto Kinova = world.addArticulatedSystem(binaryPath.getDirectory() + "\\..\\rsc\\A1\\A1\\A1_w_kinova.urdf");
  auto ground = world.addGround(0);
  world.setTimeStep(0.002);

  /// launch raisim server for visualization. Can be visualized using raisimUnity
  raisim::RaisimServer server(&world);
  server.launchServer();
  std::cout << Kinova->getDOF() << std::endl;
  // Kinova->setGeneralizedForce(Eigen::VectorXd::Zero(Kinova->getDOF()));


  raisim::Vec<3> base;
  raisim::Vec<3> position_FL;

  raisim::Vec<3> position_FR;
  raisim::Vec<3> position_RL;
  raisim::Vec<3> position_RR;
  
  for (int i=0; i<1000000; i++) {
    RS_TIMED_LOOP(world.getTimeStep()*1e6);
    server.integrateWorldThreadSafe();

    Kinova->setGeneralizedCoordinate({0.0, 0.0, 0.11, 1, 0, 0, 0,0.0, M_PI/3, -2.6, 0.0, M_PI/3, -2.6, 0.0, M_PI/3, -2.6, 0.0, M_PI/3, -2.6, 0.0, 0.9, 2.4, 0 });


    auto non_input = Kinova->getNonlinearities(world.getGravity());
    Eigen::VectorXd Nonlinearty(non_input.n);
    for(auto i =0;i< non_input.n;i++ )
    {Nonlinearty(i) = static_cast<double>(non_input[i]);}
    Nonlinearty(2) = 0.0;
    Kinova->setGeneralizedForce(Nonlinearty);
  
  if(i==1)
    { 
      std::cout<<"Iteration i : \n"<<std::endl;
      Kinova->getFramePosition("connect_root_and_world",base);      
      Kinova->getFramePosition("FL_foot_fixed",position_FL);
      Kinova->getFramePosition("RL_foot_fixed",position_FR);
      Kinova->getFramePosition("FR_foot_fixed",position_RL);
      Kinova->getFramePosition("RR_foot_fixed",position_RR);
      std::cout << base << std::endl;
      std::cout << position_FL << std::endl;
      std::cout << position_FR << std::endl;
      std::cout << position_RL << std::endl;
      std::cout << position_RR << std::endl;
     /*  std::cout << "Mass Matrix: "<<Kinova->getMassMatrix()<< std::endl;
      std::cout << "Gravity and Coriolis: "<<Kinova->getNonlinearities(world.getGravity())<< std::endl;
      std::cout << "State value: " << Kinova->getGeneralizedCoordinate()<< std::endl; */
    }

  }


  server.killServer();
}
