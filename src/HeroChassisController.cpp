#include "hero_chassis_controller/HeroChassisController.hpp"

#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(hardware_interface::EffortJointInterface* hw,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  try {
    //关节名与rm_descrption_for_task/urdf/common/mecanum_wheel.urdf.xacro
    //中wheel_transmission宏生成的"${prefix}_wheel_joint"完全一致
    front_left_joint_ = hw->getHandle("left_front_wheel_joint");
    front_right_joint_ = hw->getHandle("right_front_wheel_joint");
    back_left_joint_ = hw->getHandle("left_back_wheel_joint");
    back_right_joint_ = hw->getHandle("right_back_wheel_joint");
    cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 10,  //加上底盘控制器
                                     &HeroChassisController::cmdVelCallback, this);
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    //句柄获取失败时返回false，ControllerManager拒绝加载并上报
    //而非异常抛进实时循环里导致gzserver终止
    ROS_ERROR_STREAM("HeroChassisController:取关节句柄失败：" << e.what());
    return false;
  }

  //四个轮子的 PID：一个 PID 一个独立命名空间，否则 rqt_reconfigure 只剩一组
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
  hardware_interface::JointHandle* joints[4] = {&front_left_joint_, &front_right_joint_,
                                                &back_left_joint_, &back_right_joint_};
  control_toolbox::Pid* pids[4] = {&pid_front_left_, &pid_front_right_, &pid_back_left_,
                                   &pid_back_right_};

  constexpr double kRadius = 0.07625;    //轮半径，来自 mecanum_wheel.urdf.xacro
  constexpr double kHalfDiagonal = 0.4;  // lx + ly = 0.2 + 0.2

  const double vx = cmd_vx_, vy = cmd_vy_, wz = cmd_wz_;

  //麦克纳姆轮逆运动学：底盘坐标系速度 → 四个轮子的期望转速
  const double target[4] = {
      (vx - vy - kHalfDiagonal * wz) / kRadius,  // left_front
      (vx + vy + kHalfDiagonal * wz) / kRadius,  // right_front
      (vx + vy - kHalfDiagonal * wz) / kRadius,  // left_back
      (vx - vy + kHalfDiagonal * wz) / kRadius,  // right_back
  };

  for (int i = 0; i < 4; ++i) {
    const double measured = joints[i]->getVelocity();
    double effort = pids[i]->computeCommand(target[i] - measured, period);
    effort = std::max(-effort_limit_, std::min(effort_limit_, effort));
    joints[i]->setCommand(effort);
  }
}

//底盘坐标系，获取回调+订阅数据
void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  cmd_vx_ = msg.linear.x;
  cmd_vy_ = msg.linear.y;
  cmd_wz_ = msg.angular.z;
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

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)