#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);

  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  envir.addRobot(robot);
  double r = 0.07 ;
  double b = 0.3;
  double omegaLimit = 10;
  robot.initWheel(r, b, omegaLimit);

  Robot robot2("Alena", 0,0,0);
  envir.addRobot(robot2);
  double r2 = 0.05 ;
  double b2 = 0.3;
  double omegaLimit2 = 10;
  robot2.initWheel(r2, b2, omegaLimit2);

  RangeSensor range1(robot, 0.1, 0, 0);
  SensorBearing bearing2(robot2, 0.1, 0, 0);





  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    robot.goTo(envir.target());

    // follow the first robot
    robot2.moveWithSensor(Twist(0.4, 0, 0));

  }

  // plot trajectory
  envir.plot();

}
// ****************Incomplete methods******************
//Q1. In which file the target motion is defined?
//A1. In robot.cpp

//Q2. Explain the signature of Robot::Robot especially the way to pass arguments
//A2. Robot::Robot(string _name, double _x, double _y, double _theta) - it is class constructor,
//           the arguments define the name of the robot and its initial position (x,y) and initial twist theta

//Q3. Implement such a function in Robot::moveVW
//A3. Realized in robot.cpp

//Q4. Now that a realistic way to control the robot is possible, should the Robot::moveXYT
//    method stay available for external use? What can we do in the robot.h file to make it
//    impossible to use it from outside the Robot class
//A4. No, as nobody would control a robot by its linear speed x,y components.
//    To remove it, I put it as protected member in the robot.h file
//    I could also put it as private member, but what if we will want to make the Robot's child classes
//    (for example some specific types of robots?)

//Q5. Modify the moveWithSensor method, so that it calls the moveVW method
//A5. Realized in robot.cpp

//Q5. When a robot is equipped with two actuated wheels, a simple model is the differential
//    drive model. define new attributes in the Robot class in order to initialize the radius
//    and base distance. Create also a new method in the Robot class, called initWheel, that
//    does so.
//A5. Realized in robot.h, robot.cpp

//Q6 implement the Robot::rotateWheels
//   method, so that it can be possible to control the robot by sending wheel velocities. The
//   method should call Robot::moveXYT after having computed the (x, y, θ) velocities from
//   (ω l , ω r )
//A6. Realized in robot.cpp

//Q7. By using a bool wheels init attribute, make sure that it is impossible to do anything
//    in Robot::rotateWheels if the radius and base have not been initialized.
//A7. Realized in robot.h and robot.cpp

// ****************Velocity limits******************
//Q1. Modify the Robot::initWheels method in order to pass a new argument that defines
//   the wheel angular velocity limit, define a new attribute of the Robot
//   class to store this limit.
//A1. Realized in robot.h and robot.cpp

//Q2. Modify the Robot::rotateWheels method in order to ensure that the applied velocities
//    (ω l , ω r ) are within the bounds
//A2. Realized in robot.cpp

//Q3. Modify the Robot::moveVW method so that a (v, ω) setpoint is changed to
//    a (ω l , ω r ) setpoint that will then be called through Robot::rotateWheels.
