#include <memory>

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_listener.h>

#include "tf2/convert.h"


using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class Perception : public rclcpp_lifecycle::LifecycleNode
{
public:
  Perception(std::string robot_frame, float perception_range);

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void update_knowledge();
  void do_work();

private:
  void model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_state_sub_;
  ros2_knowledge_graph::GraphNode *knowledge_graph_;  

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string robot_frame_;
  float perception_range_;

  std::vector<std::string> objects_name_;
  std::vector<geometry_msgs::msg::Pose> objects_pose_;
};