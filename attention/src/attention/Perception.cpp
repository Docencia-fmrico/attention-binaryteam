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

  knowledge_graph_ = ros2_knowledge_graph::GraphFactory::getInstance(shared_from_this());

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
  ros2_knowledge_graph::GraphFactory::cleanUp();
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

float Perception::euclidean_distance(geometry_msgs::msg::Pose  robot_pose, geometry_msgs::msg::Pose obj_pose) {

  return sqrt( pow(robot_pose.position.x - obj_pose.position.x, 2) + pow(robot_pose.position.y - obj_pose.position.y, 2) + pow(robot_pose.position.z - obj_pose.position.z, 2));
}


void Perception::model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  std::vector<std::string>::iterator it;
  // Element to be searched
  
  geometry_msgs::msg::Pose robot_pose;
  std::string robot_name = "tiago";

  it = std::find (msg->name.begin(), msg->name.end(), robot_name);
  if (it != msg->name.end())
  {
    int position = it - msg->name.begin();
    robot_pose = msg->pose[position];
  }
  else {
    std::cout << "Robot with name " << robot_name << " not found in gazebo models.\n\n";
  }

  // Search objects near to the robot
  int i = 0;
  for (auto obj_pose: msg->pose) {

    if (euclidean_distance(robot_pose, obj_pose) < 3) {

      auto perceived_object = ros2_knowledge_graph::new_node(msg->name[i], "object");
      knowledge_graph_->update_node(perceived_object);

      geometry_msgs::msg::TransformStamped tf_robot2object;
      //tf1.transform.translation.x = 7.0;
      auto edge_tf_robot2object = ros2_knowledge_graph::new_edge(robot_name, msg->name[i], tf_robot2object);
      knowledge_graph_->update_edge(edge_tf_robot2object);

    }
    i++;
  }  
}

void Perception::update_knowledge() 
{

}

void Perception::do_work()
{

}