//
// Created by yuchen on 2022/2/9.
//
/*#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_buffer.h>*/
#include "my_controller.h"

namespace my_controller
{
    bool speed_controller::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n)   //init
        {
            std::vector<std::string> joint_names_;
            std::string param_name = "joints";
            if (!n.getParam(param_name,
                            joint_names_))                                //search joints that you have defined and check their existence
            {
                ROS_ERROR("Can't find your joints.");
                return false;
            }
            num_joints_ = joint_names_.size();

            ROS_INFO("The number of your joints: %lu", joint_names_.size());

            for (unsigned int i = 0; i < num_joints_; i++) {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            return true;
        }

        void speed_controller::update(const ros::Time &time, const ros::Duration &period) {
            double a[4] = {0.1, 0.1, 0.1, 0.1};
            std::vector<double> command(a, a + 4);
            for (unsigned int i = 0; i < num_joints_; i++) {
                joints_[i].setCommand(command[i]);
            }
        }

        void starting(const ros::Time &time);
        void stopping(const ros::Time &time);

}
PLUGINLIB_EXPORT_CLASS( my_controller::speed_controller,
                        controller_interface::ControllerBase)

/*
 *     class speed_controller :public controller_interface::Controller<hardware_interface::VelocityJointInterface>
    {
    public:
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)   //init
        {
            std::vector< std::string > joint_names_;
            std::string param_name = "joints";
            if(!n.getParam(param_name,joint_names_))                                //search joints that you have defined and check their existence
           {
                ROS_ERROR("Can't find your joints.");
                return false;
            }
            num_joints_=joint_names_.size();

            ROS_INFO("The number of your joints: %lu",joint_names_.size());

            for(unsigned int i=0; i<num_joints_; i++)
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            double a[4]={0.1,0.1,0.1,0.1};
            std::vector<double> command(a,a+4);
            for(unsigned int i=0;i<num_joints_;i++)
            {
                joints_[i].setCommand(command[i]);
            }
        }
        void starting(const ros::Time& time);
        void stopping(const ros::Time& time);

    private:
        unsigned int num_joints_;
        std::vector<hardware_interface::JointHandle> joints_;

    };


    PLUGINLIB_DECLARE_CLASS(ros_urdf_learn, speed_controller, my_controller::speed_controller, controller_interface::ControllerBase);
 */
/*
 *
    class PositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {
            // get joint name from the parameter server
            std::string my_joint;
            if (!n.getParam("joint", my_joint)){
                ROS_ERROR("Could not find joint name");
                return false;
            }

            // get the joint object to use in the realtime loop
            joint_ = hw->getHandle(my_joint);  // throws on failure
            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            double error = setpoint_ - joint_.getPosition();
            joint_.setCommand(error*gain_);
        }

        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }

    private:
        hardware_interface::JointHandle joint_;
        static const double gain_ = 1.25;
        static const double setpoint_ = 3.00;
    };
    PLUGINLIB_DECLARE_CLASS(package_name, PositionController, controller_ns::PositionController, controller_interface::ControllerBase);
 */


/*
namespace my_controller
{

    class speed_controller :public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &n)   //init
        {
            std::vector<std::string> joint_names_;
            std::string param_name = "joints";
            if (!n.getParam(param_name,
                            joint_names_))                                //search joints that you have defined and check their existence
            {
                ROS_ERROR("Can't find your joints.");
                return false;
            }
            num_joints_ = joint_names_.size();

            ROS_INFO("The number of your joints: %lu", joint_names_.size());

            for (unsigned int i = 0; i < num_joints_; i++) {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            return true;
        }

        void update(const ros::Time &time, const ros::Duration &period) {
            double a[4] = {0.1, 0.1, 0.1, 0.1};
            std::vector<double> command(a, a + 4);
            for (unsigned int i = 0; i < num_joints_; i++) {
                joints_[i].setCommand(command[i]);
            }
        }

        void starting(const ros::Time &time);

        void stopping(const ros::Time &time);

    private:
        unsigned int num_joints_;
        std::vector<hardware_interface::JointHandle> joints_;

    };
}
PLUGINLIB_EXPORT_CLASS( my_controller::speed_controller,
                        controller_interface::ControllerBase)*/