#include <memory>

#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

class Perception : public rclcpp_lifecycle::LifecycleNode
{
public:
  Perception();

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
  void model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_state_sub_;
};

