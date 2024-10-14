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

//For multithreading
#include "pthread.h"

//header helps stop while loop with ctrl+c
#include "signal.h"


int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  // auto laikago = world.addArticulatedSystem(binaryPath.getDirectory() + "\\rsc\\laikago\\laikago.urdf");
  auto A1 = world.addArticulatedSystem(binaryPath.getDirectory() + "\\..\\rsc\\A1\\A1_w_kinova.urdf");
  auto ball = world.addSphere(0.1, 1);
  auto ground = world.addGround(-2);
  world.setTimeStep(0.002);

  /// launch raisim server for visualization. Can be visualized using raisimUnity
  raisim::RaisimServer server(&world);
  server.launchServer();

  for (int i=0; i<10000; i++) {
    RS_TIMED_LOOP(world.getTimeStep()*1e6);
    server.integrateWorldThreadSafe();
    
  }

  server.killServer();
}
