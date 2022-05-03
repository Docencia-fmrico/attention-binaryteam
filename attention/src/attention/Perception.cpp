#include <vector>
#include <limits>
#include "attention/Perception.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

Perception::Perception()
: rclcpp_lifecycle::LifecycleNode("perception_node")
{
  model_state_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
    "/gazebo/model_states", 1, std::bind(&Perception::model_state_callback, this, _1));
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT Perception::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Perception::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Perception::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Perception::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Perception::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT Perception::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
  return CallbackReturnT::SUCCESS;
}


void Perception::model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  std::vector<std::string>::iterator it;
  // Element to be searched
  std::string ser = "bookshelf_4";
    
  // std::find function call
  it = std::find (msg->name.begin(), msg->name.end(), ser);
  if (it != msg->name.end())
  {
    std::cout << "Element " << ser <<" found at position : " ;
    std::cout << it - msg->name.begin() << " (counting from zero) \n" ;
    std::cout << msg->pose[it - msg->name.begin()].position.x << " " << 
      msg->pose[it - msg->name.begin()].position.y << " " << 
      msg->pose[it - msg->name.begin()].position.z << std::endl;
  }
  else
    std::cout << "Element not found.\n\n";
}

void Perception::do_work()
{

}