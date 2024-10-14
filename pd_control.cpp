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


#include <iostream>
#include <random>
using namespace std;

//header helps stop 

int main(int argc, char* argv[]) {
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);

  raisim::World world;
  std::string path = binaryPath.getDirectory() + "\\..\\rsc\\A1\\A1_w_kinova.urdf";
  std::cout<<"Path:"<<path<<std::endl;
  // auto A1 = world.addArticulatedSystem(binaryPath.getDirectory() + "..\\rsc\\A1\\A1_modified_new.urdf");
  auto A1 = world.addArticulatedSystem(path);
  auto ball = world.addSphere(0.1, 1);
  auto ground = world.addGround(-2);
  world.setTimeStep(0.002);

  raisim::RaisimServer server(&world);
  server.launchServer();

  // PD control parameters
  const size_t numJoints = A1->getDOF();
  std::cout<<"Number of Joints:"<<numJoints<<std::endl;
  const size_t gen_coordinate_dim = A1->getGeneralizedCoordinateDim();
  std::cout<<"Generalized Coordinate Dim:"<<gen_coordinate_dim<<std::endl;
  Eigen::VectorXd Kp = Eigen::VectorXd::Constant(numJoints, 200); // Proportional gains
  Eigen::VectorXd Kd = Eigen::VectorXd::Constant(numJoints, 10);  // Derivative gains
  Eigen::VectorXd desiredPosition = Eigen::VectorXd::Constant(gen_coordinate_dim,2.3); // Desired positions for each joint
  Eigen::VectorXd desiredVelocity = Eigen::VectorXd::Constant(numJoints, 0.2); // Desired velocities for each joint
  Eigen::VectorXd torques = Eigen::VectorXd::Constant(numJoints, 5.0); // Torques to be applied to each joint
//   A1->setPdGains(Kp, Kd);
//   A1->setPdTarget(desiredPosition, desiredVelocity);

  for (int i = 0; i < 100000; i++) {
    world.integrate();

    // Get the current state of the robot
    Eigen::VectorXd currentPosition, currentVelocity;
    A1->getState(currentPosition, currentVelocity);
    // A1->setPdTarget(currentPosition*1.04, currentVelocity*1.03);

   /*  // Compute the PD control torques
    Eigen::VectorXd torques(numJoints);
    for (size_t j = 0; j < numJoints; j++) {
      torques(j) = Kp[j] * (desiredPosition[j] - currentPositions(j)) + Kd[j] * (desiredVelocity[j] - currentVelocities(j));
    } */
    // Apply the torques
    //How do I set random torques?
    for (size_t j = 0; j < numJoints; j++) {
      torques(j) = (rand() % 2 + 1);
    }
    A1->setGeneralizedForce(torques);
    RS_TIMED_LOOP(world.getTimeStep()*1e6);
    // Integrate the world and update the server
    server.integrateWorldThreadSafe();
    // RS_TIMED_LOOP(world.getTimeStep()*1e6);
    // server.integrateWorldThreadSafe();
    if (i%1000==0){
        // std::cout<<"Current Positions and Velocities"<<currentPosition<<","<<currentVelocity<<std::endl;
        std::cout<<"Torques"<<torques<<std::endl;}
    
  }

  server.killServer();
  return 0;
}
