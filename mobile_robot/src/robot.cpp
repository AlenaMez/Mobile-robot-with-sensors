

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
#include <sensor_range.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}

//Q5
void Robot::initWheel(double _r, double _b, double _omegaLimit)
{
    r_ = _r;
    b_ = _b;
    omegaLimit_ = _omegaLimit;

    wheelsInit_ = true;
}

void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}



void Robot::rotateWheels(double _left, double _right)
{
    // to fill up after defining an initWheel method
    if (wheelsInit_)
    {

        auto a = max(_left/omegaLimit_, _right/omegaLimit_);

        if (a<1)
        {
            a = 1;
        }

        auto left = _left/a;
        auto right = _right/a;

        auto omega = r_*(left - right)/(2*b_);
        auto theta = pose_.theta;
        auto v = r_*(left + right)/2;
        auto vx = v*cos(theta);
        auto vy = v*sin(theta);

        moveXYT (vx, vy, omega);


    }
}


// Q3: move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{

    auto left = (_v+_omega*b_)/r_;

    auto right = (_v-_omega*b_)/r_;

    rotateWheels(left, right);

}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);
    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));

}


void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist checking

    for ( auto & sensor : sensors_ )
    {
        sensor -> updateFromRobotPose(pose_);
    }

    // uses X-Y motion (perfect but impossible in practice)
    // moveXYT(_twist.vx, _twist.vy,_twist.w);

    // Q5 to fill up, use V-W motion when defined


    for ( auto & sensor : sensors_ )
    {
        sensor -> correctRobotTwist(_twist);
    }
    std::cout << "_twist.w = "<< _twist.w << std::endl;
    std::cout << "_twist.vy = "<< _twist.vy << std::endl;
    auto alpha = 20;
    auto v = _twist.vx;
    auto omega = _twist.vy*alpha + _twist.w;

    std::cout << "omega = "<< omega << std::endl;
    moveVW(v, omega);
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

