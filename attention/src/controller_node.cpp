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

#include "geometry_msgs/msg/vector3.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class ControllerServer : public rclcpp::Node
{
public:
  ControllerServer(std::vector<std::string> to_see)
  : Node("controller_node")
  {
    service_ = create_service<controller_service_msgs::srv::Controller>(
      "controller", std::bind(&ControllerServer::service_callback, this, _1, _2));

    timer_ = create_wall_timer(1s, std::bind(&ControllerServer::selector, this));
    targets_ = to_see;
  }

  void init()
  {
    knowledge_graph_ = ros2_knowledge_graph::GraphFactory::getInstance(shared_from_this());
  }
private:

  void service_callback(const std::shared_ptr<controller_service_msgs::srv::Controller::Request> request,
          std::shared_ptr<controller_service_msgs::srv::Controller::Response>      response)
  {
    geometry_msgs::msg::Vector3 point;

    if (targets_.empty()) {
      point.x = 1;
      point.y = 0;
      point.z = 0;
    } else {
      add_want_see_edges();
      int count_times = 0;
      bool not_node = false;
      while (! knowledge_graph_->exist_node(targets_[actual_index_])) {
        selector();
        timer_->reset();
        count_times++;

        if (count_times == targets_.size()) {
          not_node = true;
          break;
        }
      }  
      if (! not_node) {
        auto edge = 
          knowledge_graph_->get_edges<geometry_msgs::msg::TransformStamped>("head_1_link", targets_[actual_index_]);
        auto content_edge =
          ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(edge[0].content);
        geometry_msgs::msg::TransformStamped tf = content_edge.value();

        point = tf.transform.translation;
        std::cerr << "----------------" << actual_index_ << std::endl;
        std::cerr << "[ " << targets_[actual_index_] << "] ====================" << std::endl;
        std::cerr << "----------------" << std::endl;
      } else {
        point.x = 1;
        point.y = 0;
        point.z = 0;
      }
      
    }

    response->point2see = point;
    
  }

  void selector()
  {
    if (targets_.empty()) {
      actual_index_ = -1;
    } else if (targets_.size() == 1) {
      actual_index_ == 0;
    } else if (actual_index_ != (targets_.size() - 1)) {
      actual_index_++;
    } else if (actual_index_ == (targets_.size() - 1)) {
      actual_index_ = 0;
    }
  }

  void add_want_see_edges()
  {
    for (auto target: targets_) {
      if (knowledge_graph_->exist_node(target) && knowledge_graph_->exist_node("head_1_link")) {
        auto edge_robot2object = ros2_knowledge_graph::new_edge<std::string>("head_1_link", target, "want_see");
        knowledge_graph_->update_edge(edge_robot2object);
      }
    }
  }

  rclcpp::Service<controller_service_msgs::srv::Controller>::SharedPtr service_;
  rclcpp::Subscription<controller_service_msgs::msg::ElementsToSee>::SharedPtr elements_to_see_sub_;
  ros2_knowledge_graph::GraphNode *knowledge_graph_;

  std::vector<std::string> targets_;
  int actual_index_ = -1; // Was -1 if targets_ vector is empty
  rclcpp::TimerBase::SharedPtr timer_;
  
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::vector<std::string> to_see = {"bookshelf_0", "bowl_0"};
  auto node = std::make_shared<ControllerServer>(to_see);
  
  node->init();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}