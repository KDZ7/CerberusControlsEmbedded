#include "iksolver/iksolver.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto lifecycle_node = std::make_shared<cerberus_iksolver::IKSolver>();
    rclcpp::spin(lifecycle_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}