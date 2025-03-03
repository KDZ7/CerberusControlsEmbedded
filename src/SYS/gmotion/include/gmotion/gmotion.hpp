#ifndef __GMOTION__
#define __GMOTION__

#include <memory>
#include <map>
#include <thread>
#include <mutex>
#include "log/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "msggroup/msg/group_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "msgwaypoint/msg/waypoint.hpp"
#include "yaml-cpp/yaml.h"

namespace cerberus_gait
{

    struct step_t
    {
        std::string leg;
        double x, y, z;
        std::vector<double> joints;
        double wait;
    };
    struct sequence_t
    {
        std::vector<std::string> sync_legs;
        std::vector<step_t> steps;
    };
    struct gait_t
    {
        std::string name;
        std::vector<sequence_t> sequences;
        std::string goto_gait;
    };

    enum GMOTION
    {
        GAIT_STOP = 0x00,
        GAIT_STAND = 0x01,
        GAIT_SIT = 0x02,
        GAIT_HI = 0x03,
        GAIT_CRAWL = 0xF0,
        GAIT_WALK = 0xF1,
        GAIT_TROT = 0xF2,
        GAIT_BOUND = 0xF3,
        GAIT_GALLOP = 0xF4,
        GAIT_TURN_LEFT = 0xF5,
        GAIT_TURN_RIGHT = 0xF6,
        GAIT_SIDE_RIGHT = 0xF7,
        GAIT_SIDE_LEFT = 0xF8,
        GAIT_BACKWARD = 0xF9
    };

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class GMotion : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        GMotion();
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

    private:
        bool loadConfig();
        void command_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void solutions_callback(const msggroup::msg::GroupState::SharedPtr msg);
        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void action(const gait_t &gait);

        std::string config_file_path_;
        std::map<std::string, gait_t> gaits_;
        std::string solver_name_;
        std::mutex solutions_mutex_;
        std::map<std::string, msggroup::msg::GroupState> solutions_cache_;

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr command_sub_;
        rclcpp::Subscription<msggroup::msg::GroupState>::SharedPtr solutions_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    };

} // namespace cerberus_gait
#endif // __GMOTION__
