//
// Created by yuchen on 2022/2/22.
//

#pragma once
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>

namespace my_controller
{
    class speed_controller :public controller_interface::Controller<hardware_interface::VelocityJointInterface>{
    public:
        bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n);
        void update(const ros::Time &time, const ros::Duration &period);

        void starting(const ros::Time &time);

        void stopping(const ros::Time &time);

    private:
        unsigned int num_joints_;
        std::vector<hardware_interface::JointHandle> joints_;
    };
}