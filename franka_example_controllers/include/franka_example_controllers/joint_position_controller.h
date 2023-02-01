// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

namespace franka_example_controllers {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg);
  double pid(double target, double current, size_t index);

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  std::array<double, 7> initial_pose_{};

  ros::Subscriber sub;


  std::vector<double> m_qGoal = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  // joint limits are set smaller than them of the real robot
  std::vector<double> m_minJointLimits = {-2.6437, -1.6837, -2.8007, -2.9421, -2.7065, 0.4445, -2.9159};
  std::vector<double> m_maxJointLimits = {2.6437, 1.6837, 2.8007, -0.0518, 2.7065, 4.4169, 2.9159};

  double m_dt = 0.001;
  double m_max = 0.0001;
  double m_min = 0.000000001;
  double m_kp = 0.0005;
  double m_kd = 0.0003;
  double m_ki = 0.00007;
  double m_accScale = 0.1;

  std::vector<double> m_preError  = std::vector<double>(7, 0);
  std::vector<double> m_integral  = std::vector<double>(7, 0);
  std::vector<double> m_lastOutput  = std::vector<double>(7, 0);
};

}  // namespace franka_example_controllers
