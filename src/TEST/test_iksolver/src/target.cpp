#include "rclcpp/rclcpp.hpp"
#include "mswaypoint/msg/waypoint.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
class Target : public rclcpp::Node
{
public:
    Target() : Node("target")
    {
        this->declare_parameter("base_frame", "base_link");
        base_frame_ = this->get_parameter("base_frame").as_string();
        waypoint_sub_ = this->create_subscription<mswaypoint::msg::Waypoint>("~/waypoint", 10, std::bind(&Target::waypoint_callback, this, std::placeholders::_1));
        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/target", 10);
    }

private:
    void waypoint_callback(const mswaypoint::msg::Waypoint::SharedPtr msg)
    {
        geometry_msgs::msg::PointStamped target;
        target.header.stamp = this->now();
        target.header.frame_id = base_frame_;
        target.point.x = msg->x;
        target.point.y = msg->y;
        target.point.z = msg->z;
        target_pub_->publish(target);
    }

    rclcpp::Subscription<mswaypoint::msg::Waypoint>::SharedPtr waypoint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    std::string base_frame_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Target>());
    rclcpp::shutdown();
    return 0;
}