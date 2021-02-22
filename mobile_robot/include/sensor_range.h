#ifndef SENSOR_RANGE_H
#define SENSOR_RANGE_H

#include <iostream>
#include <robot.h>
#include <envir.h>
#include <sensor.h>

namespace arpro
{

class RangeSensor : public Sensor
{
public : RangeSensor ( Robot & _robot , double _x , double _y , double _theta ) :
         Sensor ( _robot , _x , _y , _theta ) // call the Sensor constructor
{}

    void update(const Pose &_p)
    {
        const auto& x = _p.x;
        const auto& y = _p.y;
        const auto& theta = _p.theta;
        double min_d = 1000000;
        Pose p1 , p2 ;
        for ( int i = 0; i < envir_->walls.size(); ++i )
        {
        p1 = envir_ -> walls[i];
        p2 = envir_ -> walls[(i+1)%envir_->walls.size()];

        const auto& x1 = p1.x;
        const auto& y1 = p1.y;
        const auto& x2 = p2.x;
        const auto& y2 = p2.y;

        auto d = abs ( (x1*y2 - x1*y - x2*y1 + x2*y +x*y1 - x*y2)/(x1*sin(theta) - x2*sin(theta) - y1*cos(theta) + y2*cos(theta)) );
        if (d<min_d)
        {
            min_d = d;
        }

        }

        s_ = min_d;
        //std::cout << "The clothest wall is in " << min_d << std::endl;

    }


void correctTwist(Twist &_v)
{

    auto s_min = 0.1;
    auto g = 1;
    if (_v.vx > g*(s_ - s_min) )
    {
         _v.vx = g*(s_ - s_min);
    }
}




// the RangeSensor constructor does nothing more
};
   
}
#endif // SENSOR_RANGE_H
