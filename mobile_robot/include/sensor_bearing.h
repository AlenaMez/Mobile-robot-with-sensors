#ifndef SENSOR_BEARING_H
#define SENSOR_BEARING_H


#include <iostream>
#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <math.h>

namespace arpro
{

class SensorBearing : public Sensor
{
public : SensorBearing ( Robot & _robot , double _x , double _y , double _theta ) :
         Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
{}

    void update(const Pose &_p)
    {
        // look for first other robot
        for ( auto other : envir_ -> robots_ )
        if ( other != robot_ )
        {
            std::cout << "_p.x = " << _p.x << std::endl;
            std::cout << "_p.y = " << _p.y << std::endl;
            std::cout << "_p.theta = " << _p.theta << std::endl;


            auto p = _p.transformInverse(pose_);
           //auto p = _p;
            //auto p = _p.transformDirect(pose_);
            std::cout << "p.x = " << p.x << std::endl;
            std::cout << "p.y = " << p.y << std::endl;
            std::cout << "p.theta = " << p.theta << std::endl;

        // compute angle between sensor and detected robot
        s_ = atan2 ((other->pose().y - p.y), (other->pose().x - p.x)) - p.theta;
        break ;
        }

        // set angle back to [ - pi , pi ]
        auto n = trunc(s_/M_PI);
        if (abs(n)>0)
        {
            if (s_>0)
                s_ = s_ - 2*M_PI;
            else
                s_ = s_ + 2*M_PI;
        }

        std::cout << "s_ = " << s_ << std::endl;

    }

    void correctTwist(Twist &_v)
    {
        auto g = 0.5;
        _v.w = _v.w - g*s_;
        std::cout << "_v.w = " << _v.w << std::endl;

    }
};

}

#endif // SENSOR_BEARING_H
