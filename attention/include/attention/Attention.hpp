#include <memory>

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class Attention : public rclcpp_lifecycle::LifecycleNode
{
public:
  Attention();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void do_work();

private:
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr neck_pose_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_neck_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_neck_buffer_;

  void move_head();
};

