#include "gmotion/gmotion.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto lifecycle_node = std::make_shared<cerberus_gait::GMotion>();
    rclcpp::spin(lifecycle_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}