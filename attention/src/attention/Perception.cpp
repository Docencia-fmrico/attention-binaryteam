#include <vector>
#include <limits>
#include "attention/Perception.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


Perception::Perception(std::string robot_frame, float perception_range)
: rclcpp_lifecycle::LifecycleNode("perception_node") 
{
  model_state_sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
    "/gazebo/model_states", 100, std::bind(&Perception::model_state_callback, this, _1));
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  robot_frame_ = robot_frame;
  perception_range_ = perception_range;

  timer_ = create_wall_timer(200ms, std::bind(&Perception::do_work, this));
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT Perception::on_configure(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(
    get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());

  knowledge_graph_ = ros2_knowledge_graph::GraphFactory::getInstance(shared_from_this());

  auto robot = ros2_knowledge_graph::new_node(robot_frame_, "robot");
  knowledge_graph_->update_node(robot);

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

void Perception::model_state_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  std::cerr << "========================";
  objects_name_ = msg->name;
  objects_pose_ = msg->pose;
}

float Perception::distance_to_TF(tf2::Transform TF)
{
  tf2::Vector3 point = TF.getOrigin();

  float distance = sqrt(pow(point.getX(), 2) + pow(point.getY(), 2));

  return distance;
}

void Perception::update_knowledge() 
{ 
  std::cerr << "1";
  int i = 0;
  for (auto obj_name: objects_name_) {
    std::cerr << "2";
  geometry_msgs::msg::TransformStamped robot2odom_tf_msg;

    // GET ROBOT TO ODOM
    try
    {
      robot2odom_tf_msg = 
        tf_buffer_->lookupTransform(robot_frame_,"odom", tf2::TimePointZero);
    }
    catch(const std::exception& e)
    {
      return;
    }
    
    tf2::Stamped<tf2::Transform> robot2odom;
    tf2::fromMsg(robot2odom_tf_msg, robot2odom);

    // GET ODOM TO OBJECT
    tf2::Stamped<tf2::Transform> odom2object;
    odom2object.setOrigin(tf2::Vector3(objects_pose_[i].position.x, 
                                      objects_pose_[i].position.y,
                                      objects_pose_[i].position.z));

    tf2::Quaternion q;
    q.setRPY(objects_pose_[i].orientation.x, objects_pose_[i].orientation.y, objects_pose_[i].orientation.z);
    odom2object.setRotation(q);

    tf2::Transform robot2object = robot2odom * odom2object;
    geometry_msgs::msg::TransformStamped robot2object_tf_msg;

    robot2object_tf_msg.header.stamp = now();
    robot2object_tf_msg.header.frame_id = robot_frame_;
    robot2object_tf_msg.child_frame_id = obj_name;
    robot2object_tf_msg.transform = tf2::toMsg(robot2object);


    if (distance_to_TF(robot2object) < perception_range_) {
      std::cerr << "3";
      if (obj_name != "tiago" && obj_name != "ground_plane") {
        // Add to knowledge
        std::cerr << "4" << std::endl;
        auto perceived_object = ros2_knowledge_graph::new_node(obj_name, "object");
        knowledge_graph_->update_node(perceived_object);

        auto edge_robot2object_tf = ros2_knowledge_graph::new_edge<geometry_msgs::msg::TransformStamped>(robot_frame_, obj_name, robot2object_tf_msg);
        knowledge_graph_->update_edge(edge_robot2object_tf); 
      }

    } else {
      std::cerr << "5";
      knowledge_graph_->remove_node(obj_name);
    }

    i++;
  } 

} 

void Perception::do_work()
{
  update_knowledge();
}