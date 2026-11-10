#pragma once

//ROS Control核心头文件
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>

namespace hero_chassis_controller {

    class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        //四个核心函数
        bool init(hardware_interface::EffortJointInterface* hw,
                  ros::NodeHandle& root_nh,
                  ros::NodeHandle& controller_nh) override;

        void update(const ros::Time& time, const ros::Duration& period) override;
        void starting(const ros::Time& time) override;
        void stopping(const ros::Time& time) override;

    private:
        //四个轮子的关节句柄
        hardware_interface::JointHandle front_left_joint_;
        hardware_interface::JointHandle front_right_joint_;
        hardware_interface::JointHandle back_left_joint_;
        hardware_interface::JointHandle back_right_joint_;

        //四个轮子的 PID 控制器
        control_toolbox::Pid pid_front_left_;
        control_toolbox::Pid pid_front_right_;
        control_toolbox::Pid pid_back_left_;
        control_toolbox::Pid pid_back_right_;

        //力矩上限（N·m），从 yaml 的 effort_limit 读
        double effort_limit_{1.0};
    };

} // namespace hero_chassis_controller