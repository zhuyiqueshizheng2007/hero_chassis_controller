#include"hero_chassis_controller/HeroChassisController.hpp"
#include<pluginlib/class_list_macros.hpp>
#include<algorithm>

namespace hero_chassis_controller {

    bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw,
                                     ros::NodeHandle& root_nh,
                                     ros::NodeHandle& controller_nh) {
        try{
            //关节名与rm_descrption_for_task/urdf/common/mecanum_wheel.urdf.xacro
            //中wheel_transmission宏生成的"${prefix}_wheel_joint"完全一致
            front_left_joint_ = hw->getHandle("left_front_wheel_joint");
            front_right_joint_ = hw->getHandle("right_front_wheel_joint");
            back_left_joint_ = hw->getHandle("left_back_wheel_joint");
            back_right_joint_ = hw->getHandle("right_back_wheel_joint");
        }catch (const hardware_interface::HardwareInterfaceException& e)
        {
            //句柄获取失败时返回false，ControllerManager拒绝加载并上报
            //而非异常抛进实时循环里导致gzserver终止
            ROS_ERROR_STREAM("HeroChassisController:取关节句柄失败："<<e.what());
            return false;
        }

        // 四个轮子的 PID：一个 PID 一个独立命名空间，否则 rqt_reconfigure 只剩一组
        const std::string wheel_names[4] = {"left_front", "right_front", "left_back", "right_back"};
        control_toolbox::Pid* pids[4] = {&pid_front_left_, &pid_front_right_, &pid_back_left_,
                                         &pid_back_right_};
        for (int i = 0; i < 4; ++i) {
            ros::NodeHandle pid_nh(controller_nh, wheel_names[i]);
            if (!pids[i]->init(pid_nh)) {
                ROS_ERROR_STREAM("PID 参数缺失: " << pid_nh.getNamespace());
                return false;
            }
        }

        if (!controller_nh.getParam("effort_limit", effort_limit_)) {
            ROS_WARN("未设置 effort_limit，使用默认 1.0");
        }

        return true;
    }

    void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
        //目标轮速（rad/s），选做5，之前先用常量，方便调 PID
        constexpr double kTargetVelocity = 3.0;

        hardware_interface::JointHandle* joints[4] = {&front_left_joint_, &front_right_joint_,
                                                      &back_left_joint_, &back_right_joint_};
        control_toolbox::Pid* pids[4] = {&pid_front_left_, &pid_front_right_, &pid_back_left_,
                                         &pid_back_right_};

        for (int i = 0; i < 4; ++i) {
            const double measured = joints[i]->getVelocity();
            const double error = kTargetVelocity - measured;         // target - state
            double effort = pids[i]->computeCommand(error, period);
            effort = std::max(-effort_limit_, std::min(effort_limit_, effort));  // 输出限幅
            joints[i]->setCommand(effort);
        }
    }

    void HeroChassisController::starting(const ros::Time& time) {
        //清空积分和微分历史，避免带着上一次的残余量启动
        pid_front_left_.reset();
        pid_front_right_.reset();
        pid_back_left_.reset();
        pid_back_right_.reset();
    }

    void HeroChassisController::stopping(const ros::Time& time) {
        //停用时力矩归零，别让车带着残余力矩
        front_left_joint_.setCommand(0.0);
        front_right_joint_.setCommand(0.0);
        back_left_joint_.setCommand(0.0);
        back_right_joint_.setCommand(0.0);
    }

} // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)