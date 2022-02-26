//
// Created by yuchen on 2022/2/25.
//

#ifndef TRIANGLE_BASE_H_
#define TRIANGLE_BASE_H_
#include "pluginlib_learn/base.h"
#include "cmath"

namespace regular_plugins
{
    class triangle : public base_class::regular
    {
    public:
        triangle():side_length_(){}

        void initialize(double side_length)
        {
            side_length_ = side_length;
        }

        double square()
        {
            return 0.5*side_length_*getHeight();
        }

        //triangle's interface that only it has
        double getHeight()
        {
            return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
        }

    private:
        double side_length_;
    };
}

#endif
