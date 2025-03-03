#include <map>
#include "log/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "msggroup/msg/group_state.hpp"
#include "yaml-cpp/yaml.h"

class IKSolver_Bridge : public rclcpp::Node
{
public:
    IKSolver_Bridge() : Node("iksolver_bridge")
    {
        this->declare_parameter<std::string>("config_file_path", "");
        std::string config_file_path = this->get_parameter("config_file_path").as_string();
        try
        {
            YAML::Node config = YAML::LoadFile(config_file_path);
            YAML::Node mimic_masters = config["mimic_masters"];
            for (const auto &master : mimic_masters)
                mimic_masters_[master.first.as<std::string>()] = master.second.as<std::string>();
            YAML::Node mimic_multipliers = config["mimic_multipliers"];
            for (const auto &multiplier : mimic_multipliers)
                mimic_multipliers_[multiplier.first.as<std::string>()] = multiplier.second.as<double>();
            YAML::Node mimic_offsets = config["mimic_offsets"];
            for (const auto &offset : mimic_offsets)
                mimic_offsets_[offset.first.as<std::string>()] = offset.second.as<double>();
            YAML::Node mimic_limits = config["mimic_limits"];
            for (const auto &limit : mimic_limits)
            {
                std::string name = limit.first.as<std::string>();
                double min = limit.second["min"].as<double>();
                double max = limit.second["max"].as<double>();
                mimic_limits_[name] = {min, max};
            }

            LOG_INFO("IKSolver_Bridge configuration loaded successfully.");
        }
        catch (const YAML::Exception &e)
        {
            LOG_ERROR("IKSolver_Bridge failed to load configuration from: " << config_file_path);
            return;
        }

        solution_sub_ = this->create_subscription<msggroup::msg::GroupState>("~/in", rclcpp::QoS(10), std::bind(&IKSolver_Bridge::solution_callback, this, std::placeholders::_1));
        solutions_pub_ = this->create_publisher<msggroup::msg::GroupState>("~/out", rclcpp::QoS(10));
    }

private:
    double wrap_symmetric(double rad, double min, double max)
    {
        if (min >= max)
            return rad;

        const double range = max - min;
        const double range2 = 2.0 * range;

        double normalized = std::fmod(rad - min, range2);
        if (normalized < 0)
            normalized += range2;

        if (normalized > range)
            normalized = range2 - normalized;

        return min + normalized;
    }

    void solution_callback(const msggroup::msg::GroupState::SharedPtr msg)
    {
        msggroup::msg::GroupState group_state = *msg;

        for (const auto &[master, slave] : mimic_masters_)
        {
            auto slave_it = std::find(group_state.name.begin(), group_state.name.end(), slave);
            if (slave_it == group_state.name.end())
                continue;

            size_t slave_index = std::distance(group_state.name.begin(), slave_it);
            double multiplier = mimic_multipliers_[master];
            double offset = mimic_offsets_[master];
            double master_value = (group_state.position[slave_index] - offset) / multiplier;

            auto mimic_limit_it = mimic_limits_.find(master);
            if (mimic_limit_it != mimic_limits_.end())
            {
                double min = mimic_limit_it->second.first;
                double max = mimic_limit_it->second.second;
                master_value = wrap_symmetric(master_value, min, max);
            }

            auto master_it = std::find(group_state.name.begin(), group_state.name.end(), master);
            if (master_it != group_state.name.end())
            {
                size_t master_index = std::distance(group_state.name.begin(), master_it);
                group_state.position[master_index] = master_value;
            }
            else
            {
                group_state.name.push_back(master);
                group_state.position.push_back(master_value);
                group_state.velocity.push_back(0.0);
                group_state.effort.push_back(0.0);
            }
        }

        solutions_pub_->publish(group_state);
    }

    rclcpp::Subscription<msggroup::msg::GroupState>::SharedPtr solution_sub_;
    rclcpp::Publisher<msggroup::msg::GroupState>::SharedPtr solutions_pub_;
    std::map<std::string, std::string> mimic_masters_;
    std::map<std::string, double> mimic_multipliers_;
    std::map<std::string, double> mimic_offsets_;
    std::map<std::string, std::pair<double, double>> mimic_limits_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKSolver_Bridge>());
    rclcpp::shutdown();
    return 0;
}
