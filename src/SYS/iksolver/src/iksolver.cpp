#include "iksolver/iksolver.hpp"
#include <limits>
#include <ctime>

namespace cerberus_iksolver
{
    IKSolver::IKSolver() : LifecycleNode("iksolver")
    {
        this->declare_parameter("config_file_path", "");
        this->declare_parameter("urdf_content", "");

        solver_.grid.max_iter = 200;
        solver_.grid.delta = 1e-3;
        solver_.grid.radius = 1e-2;
        solver_.grid.enable_refinement = true;
        solver_.grid.random_seed = -1;

        LOG_INFO("IKSolver created.");
    }

    CallbackReturn IKSolver::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("IKSolver configuring ...");

        config_file_path_ = this->get_parameter("config_file_path").as_string();
        urdf_content_ = this->get_parameter("urdf_content").as_string();

        if (config_file_path_.empty() || urdf_content_.empty())
        {
            LOG_ERROR("<config_file_path> and <urdf_content> parameters must be set.");
            return CallbackReturn::FAILURE;
        }

        if (!loadConfig() || !loadModel())
        {
            LOG_ERROR("Failed to load configuration or model.");
            return CallbackReturn::FAILURE;
        }

        LOG_INFO("IKSolver configured successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn IKSolver::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("IKSolver activating ...");

        waypoint_sub_ = this->create_subscription<mswaypoint::msg::Waypoint>("~/waypoint", rclcpp::QoS(10), std::bind(&IKSolver::waypoint_callback, this, std::placeholders::_1));
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("~/joint_states", rclcpp::QoS(10), std::bind(&IKSolver::joint_state_callback, this, std::placeholders::_1));
        solutions_pub_ = this->create_publisher<msgroup::msg::GroupState>("~/solutions", rclcpp::QoS(10));

        LOG_INFO("IKSolver activated successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn IKSolver::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("IKSolver deactivating ...");

        waypoint_sub_.reset();
        joint_states_sub_.reset();
        solutions_pub_.reset();

        LOG_INFO("IKSolver deactivated successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn IKSolver::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("IKSolver cleaning up ...");

        groups_.clear();
        kdl_joint_states_.clear();

        LOG_INFO("IKSolver cleaned up successfully.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn IKSolver::on_error([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("IKSolver encountered an error.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn IKSolver::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("IKSolver shutdown successfully.");
        return CallbackReturn::SUCCESS;
    }

    bool IKSolver::loadConfig()
    {
        LOG_INFO("IKSolver loading configuration from: " << config_file_path_);
        YAML::Node config = YAML::LoadFile(config_file_path_);
        try
        {
            if (!config["groups"])
            {
                LOG_ERROR("No groups found in configuration.");
                return false;
            }
            YAML::Node groups = config["groups"];
            for (const auto &group : groups)
            {
                std::string group_name = group.first.as<std::string>();
                YAML::Node content = group.second;
                group_t new_group;
                new_group.name = group_name;
                new_group.base_link = content["base_link"][0].as<std::string>();
                new_group.end_link = content["end_link"][0].as<std::string>();
                for (const auto &joint : content["joints"])
                {
                    std::string joint_name = joint.as<std::string>();
                    new_group.joints.push_back(joint_name);
                    YAML::Node limits = content["limits"][joint_name];
                    new_group.limits[joint_name]["min"] = limits["min"].as<double>();
                    new_group.limits[joint_name]["max"] = limits["max"].as<double>();
                }
                for (const auto &length : content["lengths"])
                    new_group.lengths.push_back(length.as<double>());
                groups_.emplace(group_name, std::move(new_group));
                kdl_joint_states_[group_name] = KDL::JntArray(groups_[group_name].joints.size());
            }

            if (config["solvers"])
            {
                YAML::Node solvers = config["solvers"];
                if (solvers["grid"])
                {
                    auto grid = solvers["grid"];
                    solver_.grid.max_iter = grid["max_iter"] ? grid["max_iter"].as<int>() : solver_.grid.max_iter;
                    solver_.grid.delta = grid["delta"] ? grid["delta"].as<double>() : solver_.grid.delta;
                    solver_.grid.radius = grid["radius"] ? grid["radius"].as<double>() : solver_.grid.radius;
                    solver_.grid.enable_refinement = grid["enable_refinement"] ? grid["enable_refinement"].as<bool>() : solver_.grid.enable_refinement;
                    solver_.grid.random_seed = grid["random_seed"] ? grid["random_seed"].as<int>() : solver_.grid.random_seed;
                }
            }
            else
                LOG_WARN("No solvers block found in configuration. Default solver parameters will be used.");

            LOG_INFO("IKSolver configuration loaded successfully.");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Failed to load configuration: " << e.what());
            return false;
        }
    }

    bool IKSolver::loadModel()
    {
        LOG_INFO("IKSolver loading URDF model ...");
        try
        {
            if (urdf_content_.empty())
            {
                LOG_ERROR("URDF content parameter is empty.");
                return false;
            }
            urdf::Model robot_model;
            if (!robot_model.initString(urdf_content_))
            {
                LOG_ERROR("Failed to load URDF model from content.");
                return false;
            }
            KDL::Tree tree;
            if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
            {
                LOG_ERROR("Failed to extract KDL tree from URDF model.");
                return false;
            }
            for (auto &group : groups_)
            {
                if (!tree.getChain(group.second.base_link, group.second.end_link, group.second.chain))
                {
                    LOG_ERROR("Failed to extract KDL chain from URDF model.");
                    return false;
                }
                group.second.fksolver = std::make_unique<KDL::ChainFkSolverPos_recursive>(group.second.chain);
            }
            for (const auto &group : groups_)
            {
                std::stringstream chain_display;
                chain_display << "> Chain [ " << group.first << " ] : " << group.second.base_link << " ---> ";
                for (size_t i = 0; i < group.second.chain.getNrOfSegments(); i++)
                    chain_display << "[" << group.second.chain.getSegment(i).getName() << "] ---> ";
                chain_display << group.second.end_link;
                LOG_INFO(chain_display.str());
            }
            LOG_INFO("IKSolver URDF model loaded successfully.");
            return true;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Failed to load URDF model: " << e.what());
            return false;
        }
    }

    void IKSolver::waypoint_callback(const mswaypoint::msg::Waypoint::SharedPtr msg)
    {
        LOG_INFO("Waypoint received [ " << msg->group << " / " << msg->solver << " ] : (" << msg->x << ", " << msg->y << ", " << msg->z << ")");
        auto group = groups_.find(msg->group);
        if (group == groups_.end())
        {
            LOG_ERROR("Group not found: " << msg->group);
            return;
        }
        const group_t &selected_group = group->second;
        KDL::Vector target(msg->x, msg->y, msg->z);
        KDL::JntArray joints;
        if (msg->solver == "GRID")
            joints = solveGRID(selected_group, target);
        else
        {
            LOG_ERROR("Solver not found: " << msg->solver);
            return;
        }
        if (joints.rows() == 0)
        {
            LOG_ERROR("IK Solver failed for [ " << msg->group << " / " << msg->solver << " ] : (" << msg->x << ", " << msg->y << ", " << msg->z << ")");
            return;
        }
        msgroup::msg::GroupState group_state;
        group_state.header = msg->header;
        group_state.group = msg->group;
        group_state.name = selected_group.joints;
        group_state.position.resize(joints.rows(), 0.0);
        group_state.velocity.resize(joints.rows(), 0.0);
        group_state.effort.resize(joints.rows(), 0.0);
        for (size_t i = 0; i < joints.rows(); i++)
            group_state.position[i] = joints(i);
        solutions_pub_->publish(group_state);
        std::stringstream solution_display;
        solution_display << "IK Solver for [ " << msg->group << " / " << msg->solver << " ] : ( " << msg->x << ", " << msg->y << ", " << msg->z << " ) ---> ( ";
        for (size_t i = 0; i < joints.rows(); i++)
        {
            solution_display << joints(i);
            if (i != joints.rows() - 1)
                solution_display << ", ";
        }
        solution_display << " )";
        LOG_INFO(solution_display.str());
    }

    void IKSolver::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->name.size(); i++)
        {
            const auto &joint_name = msg->name[i];
            double joint_position = msg->position[i];
            for (const auto &group : groups_)
            {
                const auto &group_name = group.first;
                const auto &joints = group.second.joints;
                auto joint = std::find(joints.begin(), joints.end(), joint_name);
                if (joint != joints.end())
                {
                    size_t joint_index = std::distance(joints.begin(), joint);
                    kdl_joint_states_[group_name](joint_index) = joint_position;
                }
            }
        }
    }

    KDL::JntArray IKSolver::solveGRID(const group_t &group, const KDL::Vector &target)
    {
        const int max_iter = solver_.grid.max_iter;
        double delta = solver_.grid.delta;
        double radius = solver_.grid.radius;
        const bool enable_refinement = solver_.grid.enable_refinement;
        const int dof = group.joints.size();

        if (solver_.grid.random_seed == -1)
            std::srand(std::time(nullptr));
        else
            std::srand(solver_.grid.random_seed);

        KDL::JntArray current_q = kdl_joint_states_[group.name];
        KDL::JntArray q = current_q;
        KDL::JntArray min_limits(dof), max_limits(dof);
        for (int i = 0; i < dof; i++)
        {
            min_limits(i) = group.limits.at(group.joints[i]).at("min");
            max_limits(i) = group.limits.at(group.joints[i]).at("max");
        }
        double min_error = std::numeric_limits<double>::max();
        for (int level = 0; level < (enable_refinement ? 2 : 1); level++)
        {
            for (int iter = 0; iter < max_iter; iter++)
            {
                KDL::JntArray candidate_q(dof);
                for (int i = 0; i < dof; i++)
                {
                    double offset = ((std::rand() / double(RAND_MAX)) * 2.0 - 1.0) * radius;
                    double value = current_q(i) + offset;
                    candidate_q(i) = std::max(min_limits(i), std::min(max_limits(i), value));
                }
                KDL::Frame candidate_frame;
                group.fksolver->JntToCart(candidate_q, candidate_frame);
                double error = (target - candidate_frame.p).Norm();
                if (error < min_error)
                {
                    min_error = error;
                    q = candidate_q;
                }
            }
            if (enable_refinement)
            {
                current_q = q;
                delta /= 2.0;
                radius /= 2.0;
            }
        }
        KDL::Frame solution_frame;
        group.fksolver->JntToCart(q, solution_frame);
        double final_error = (target - solution_frame.p).Norm();
        LOG_INFO("IKSolver GRID converged. Final error: " << final_error);
        return q;
    }
} // namespace cerberus_iksolver
