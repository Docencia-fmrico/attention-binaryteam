#include <vector>
#include <limits>
#include "attention/Attention.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

Attention::Attention()
: rclcpp_lifecycle::LifecycleNode("attention_node")
{
  neck_pose_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/head_controller/joint_trajectory", 100);

  tf_neck_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_neck_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_neck_buffer_);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT Attention::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Attention::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());

  neck_pose_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Attention::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());

  neck_pose_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Attention::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
  neck_pose_pub_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Attention::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
  neck_pose_pub_.reset();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Attention::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

void Attention::move_head()
{
  trajectory_msgs::msg::JointTrajectory msg;
  std::vector<std::string> joints {"head_1_joint", "head_2_joint"};
  msg.header.stamp = now();
  msg.joint_names.resize(2);
  msg.joint_names = joints;
  msg.points.resize(1);
  msg.points[0].positions.resize(2);
  msg.points[0].velocities.resize(2);
  msg.points[0].accelerations.resize(2);
  msg.points[0].effort.resize(2);
  msg.points[0].positions[0] = -1.57;
  msg.points[0].positions[1] = 0.0;
  msg.points[0].velocities[0] = 0.1;
  msg.points[0].velocities[1] = 0.0;
  msg.points[0].accelerations[0] = 0.1;
  msg.points[0].accelerations[1] = 0.0;
  msg.points[0].effort[0] = 0.1;
  msg.points[0].effort[1] = 0.1;
  msg.points[0].time_from_start = rclcpp::Duration(1);

  neck_pose_pub_->publish(msg);

}

void Attention::do_work()
{
  move_head();
}