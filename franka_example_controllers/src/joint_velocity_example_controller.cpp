// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {  
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }


  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  // try {
  //   auto state_handle = state_interface->getHandle(arm_id + "_robot");

  //   std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  //   for (size_t i = 0; i < q_start.size(); i++) {
  //     if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
  //       ROS_ERROR_STREAM(
  //           "JointVelocityExampleController: Robot is not in the expected starting position for "
  //           "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //           "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //       return false;
  //     }
  //   }
  // } catch (const hardware_interface::HardwareInterfaceException& e) {
  //   ROS_ERROR_STREAM(
  //       "JointVelocityExampleController: Exception getting state handle: " << e.what());
  //   return false;
  // }

  ros::NodeHandle n;
  sub = n.subscribe("desired_joints", 5, &JointVelocityExampleController::jointsCallback, this);

  return true;
}

void JointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void JointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  double cur, vel;
  for (size_t index = 0; index < 7; ++index) { 
      cur = velocity_joint_handles_[index].getPosition();
      vel = pid(m_qGoal[index], cur, index);
      velocity_joint_handles_[index].setCommand(vel);
  }
}

void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

double JointVelocityExampleController::pid(double target, double current, size_t index) {
  double error = (target - current);
    
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

  return output;
}

void JointVelocityExampleController::jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  // check the limits
  if (msg->position.size() < 7 || msg->position.size() > 9) {
    ROS_WARN("Wrong input dimension");
    return;
  }

  const auto &pos = msg->position;

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

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
