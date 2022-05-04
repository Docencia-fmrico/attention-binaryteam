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
void Attention::look_at(geometry_msgs::msg::TransformStamped object_tf) {

  geometry_msgs::msg::TransformStamped tf_neck2object;
  
  try {
    tf_neck2object = tf_neck_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s",
      toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }

}

void Attention::do_work()
{

}