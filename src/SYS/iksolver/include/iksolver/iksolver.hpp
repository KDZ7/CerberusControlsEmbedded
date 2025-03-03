#ifndef __IKSOLVER__
#define __IKSOLVER__

#include <memory>
#include <string>
#include <vector>
#include <map>
#include "log/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "msggroup/msg/group_state.hpp"
#include "msgwaypoint/msg/waypoint.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "urdf/model.h"

namespace cerberus_iksolver
{
    struct group_t
    {
        std::string name;
        std::string base_link;
        std::string end_link;
        std::vector<std::string> joints;
        std::vector<double> lengths;
        std::map<std::string, std::map<std::string, double>> limits;
        KDL::Chain chain;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fksolver;
    };

    struct solver_t
    {
        struct grid_t
        {
            int max_iter;
            double delta;
            double radius;
            bool enable_refinement;
            int max_refinement_iter;
            int random_seed;
        } grid;
    };

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    class IKSolver : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        IKSolver();
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state);

    private:
        bool loadConfig();
        bool loadModel();
        void waypoint_callback(const msgwaypoint::msg::Waypoint::SharedPtr msg);
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        KDL::JntArray solveGRID(const group_t &group, const KDL::Vector &target);

        std::string config_file_path_;
        std::string urdf_content_;
        std::map<std::string, group_t> groups_;
        solver_t solver_;
        std::map<std::string, KDL::JntArray> kdl_joint_states_;

        rclcpp::Subscription<msgwaypoint::msg::Waypoint>::SharedPtr waypoint_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
        rclcpp::Publisher<msggroup::msg::GroupState>::SharedPtr solutions_pub_;
    };
} // namespace cerberus_iksolver

#endif // __IKSOLVER__
