//
// Created by yuchen on 2022/2/25.
//

#include "boost/shared_ptr.hpp"

#include "pluginlib/class_loader.h"
#include "pluginlib_learn/base.h"

int main(int argc,char** argv)
{

    // 创建一个ClassLoader，用来加载plugin
    pluginlib::ClassLoader<base_class::regular> loader("pluginlib_learn", "base_class::regular");

    try {
        //load class
        boost::shared_ptr<base_class::regular> triangle = loader.createInstance("regular_plugins/triangle");

        triangle->initialize(50.0);

        ROS_INFO("square: %.3f",triangle->square());
    }
    catch (pluginlib::PluginlibException& ex)
    {
        ROS_ERROR("Error: %s",ex.what());
    }

    return 0;
}