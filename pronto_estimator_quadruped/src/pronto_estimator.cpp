#include <rclcpp/rclcpp.hpp> 

#include "pronto_estimator_quadruped/pronto_node.hpp"

using namespace pronto;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // RCLCPP_INFO(rclcpp::get_logger("INIZIO"), "------------- PRONTO NODE HAS STARTED -------------");
    auto node = std::make_shared<estimator_quad::ProntoNode<sensor_msgs::msg::JointState>>();
    node->run();

    return 0;
}
  
