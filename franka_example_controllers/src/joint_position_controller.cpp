// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_controller.h>

#include <cmath>
#include <thread>
#include <chrono>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
      m_qGoal[i] = position_joint_handles_[i].getPosition();
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  ros::NodeHandle n;
  sub = n.subscribe("desired_joints", 5, &JointPositionController::jointsCallback, this);

  return true;
} 

void JointPositionController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) 
    initial_pose_[i] = position_joint_handles_[i].getPosition();
}

void JointPositionController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  double cur, diff;
  for (size_t index = 0; index < 7; ++index) { 
      cur = position_joint_handles_[index].getPosition();
      diff = pid(m_qGoal[index], cur, index);
      position_joint_handles_[index].setCommand(cur + diff);
  }
}

double JointPositionController::pid(double target, double current, size_t index) {
  double error = target - current;
    
  double pOut = m_kp * error; // Proportional term

  // Integral term
  m_integral[index] += error * m_dt;
  double iOut = m_ki * m_integral[index];

  // Derivative term
  double derivative = (error - m_preError[index]) / m_dt;
  double dOut = m_kd * derivative;
    
  double output = pOut + iOut + dOut; // Calculate total output

  // Restrict to max/min
  if(output > m_max)
    output = m_max;
  else if(output < -m_max)
    output = -m_max;
  else if(std::abs(output) < m_min)
    output = 0;

  m_preError[index] = error; // Save error to previous error
  
  auto diff = m_lastOutput[index] - output ;
  if (output != 0 && std::abs(error) > 0.01) 
    if (std::abs(diff) > 0.001 || m_lastOutput[index] == 0)
      m_lastOutput[index] = m_lastOutput[index] + diff * m_accScale;
  m_lastOutput[index] = output; // Save to last output

  return output;
}

void JointPositionController::jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // check the limits
  if (msg->position.size() < 7 || msg->position.size() > 9) {
    ROS_WARN("Wrong input dimension");
    return;
  }

  const auto &pos = msg->position;
  for (size_t i = 0; i < 7; ++i)
    if (pos[i] < m_minJointLimits[i] || m_maxJointLimits[i] < pos[i]) {
      ROS_WARN("Out of limits of joint [%i]", static_cast<int>(i));
      return;
    }

  // check the difference
  double diff = 0;
  for (size_t i = 0; i < 7; ++i)
    diff += std::abs(pos[i] - m_qGoal[i]);
  if (diff > 1) {
    ROS_INFO("reset the integral and error term");
    m_preError = std::vector<double>(7, 0);
    m_integral = std::vector<double>(7, 0);
    m_lastOutput = std::vector<double>(7, 0);
  }
  
  m_qGoal = pos;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionController,
                       controller_interface::ControllerBase)
