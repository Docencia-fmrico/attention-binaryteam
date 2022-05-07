#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "controller_service_msgs/srv/controller.hpp"
#include "controller_service_msgs/msg/elements_to_see.hpp"

#include "ros2_knowledge_graph_msgs/msg/graph.hpp"
#include "ros2_knowledge_graph_msgs/msg/node.hpp"
#include "ros2_knowledge_graph_msgs/msg/edge.hpp"
#include "ros2_knowledge_graph_msgs/msg/graph_update.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ControllerServer : public rclcpp::Node
{
public:
  ControllerServer()
  : Node("controller_node")
  {
    service_ = create_service<controller_service_msgs::srv::Controller>(
      "controller", std::bind(&ControllerServer::service_callback, this, _1, _2));

    elements_to_see_sub_ = create_subscription<controller_service_msgs::msg::ElementsToSee>(
    "/elements_to_see", 1, std::bind(&ControllerServer::fill_list_callback, this, _1));
  }

  void init()
  {
    knowledge_graph_ = ros2_knowledge_graph::GraphFactory::getInstance(shared_from_this());
  }

  void service_callback(const std::shared_ptr<controller_service_msgs::srv::Controller::Request> request,
          std::shared_ptr<controller_service_msgs::srv::Controller::Response>      response)
  {
    /*
    if (targets_.empty()) {
      response->error = true;
    } else {
      std::vector<ros2_knowledge_graph_msgs::msg::Node> nodes = knowledge_graph_->get_nodes();
      if (targets_[actual_index_])
    }
    */
  }

  void fill_list_callback(const controller_service_msgs::msg::ElementsToSee::SharedPtr msg)
  { 
    targets_ = msg->types_elements;
  }

private:
  rclcpp::Service<controller_service_msgs::srv::Controller>::SharedPtr service_;
  rclcpp::Subscription<controller_service_msgs::msg::ElementsToSee>::SharedPtr elements_to_see_sub_;
  ros2_knowledge_graph::GraphNode *knowledge_graph_;

  std::vector<std::string> targets_;
  int actual_index_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ControllerServer>();
  
  node->init();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}