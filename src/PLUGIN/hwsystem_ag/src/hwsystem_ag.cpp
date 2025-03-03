#include "hwsystem_ag/hwsystem_ag.hpp"
#include "yaml-cpp/yaml.h"
#include "pluginlib/class_loader.hpp"

#define DEFAULT_REAL_STATE_TIMEOUT 1000.0
#define DEFAULT_REAL_STATE_PERIOD 1000.0
#define DEFAULT_MIN_POSITION -0.785398163
#define DEFAULT_MAX_POSITION 0.785398163397

namespace hwsystem_ag
{
    CallbackReturn HwsystemAg::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        try
        {
            if (info_.hardware_parameters.find("config_file_path") == info_.hardware_parameters.end())
            {
                LOG_ERROR("Parameter 'config_file_path' missing in configuration");
                return CallbackReturn::ERROR;
            }

            config_file_path_ = info_.hardware_parameters.at("config_file_path");
            YAML::Node hwconfig = YAML::LoadFile(config_file_path_);
            if (!hwconfig["idmap"])
            {
                LOG_ERROR("idmap section missing in configuration file");
                return CallbackReturn::ERROR;
            }

            YAML::Node idmap = hwconfig["idmap"];
            LOG_INFO("Joint ID mapping:");
            for (const auto &joint : idmap)
            {
                joint_idmap_[joint.first.as<std::string>()] = joint.second.as<uint8_t>();
                LOG_INFO("> Joint: " << joint.first.as<std::string>() << " <=> ID: " << (int)joint.second.as<uint8_t>());
            }

            LOG_INFO("Loaded (" << joint_idmap_.size() << ") joint ID mappings from configuration file: " << config_file_path_);
            LOG_INFO("Joint settings:");

            for (const auto &joint : info.joints)
            {
                if (!joint.state_interfaces.empty())
                    joint_state_positions_[joint.name] = 0.0;
                if (!joint.command_interfaces.empty())
                {
                    joint_command_positions_[joint.name] = 0.0;
                    for (const auto &limit : joint.command_interfaces)
                    {
                        if (limit.name == hardware_interface::HW_IF_POSITION)
                        {
                            if (limit.parameters.find("max") != limit.parameters.end())
                            {
                                joint_max_positions_[joint.name] = std::stod(limit.parameters.at("max"));
                                LOG_INFO("> Joint: " << joint.name << " Max Position: " << joint_max_positions_[joint.name]);
                            }
                            else
                            {
                                joint_max_positions_[joint.name] = DEFAULT_MAX_POSITION;
                                LOG_WARN("> Joint: " << joint.name << " Max Position not found, using default: " << joint_max_positions_[joint.name]);
                            }
                            if (limit.parameters.find("min") != limit.parameters.end())
                            {
                                joint_min_positions_[joint.name] = std::stod(limit.parameters.at("min"));
                                LOG_INFO("> Joint: " << joint.name << " Min Position: " << joint_min_positions_[joint.name]);
                            }
                            else
                            {
                                joint_min_positions_[joint.name] = DEFAULT_MIN_POSITION;
                                LOG_WARN("> Joint: " << joint.name << " Min Position not found, using default: " << joint_min_positions_[joint.name]);
                            }
                        }
                    }
                }
            }
            if (hwconfig["home_positions"])
            {
                YAML::Node offsets = hwconfig["home_positions"];
                for (const auto &joint : offsets)
                {
                    home_positions_[joint.first.as<std::string>()] = joint.second.as<double>();
                    LOG_INFO("> Joint: " << joint.first.as<std::string>() << " home position: " << joint.second.as<double>());
                }
            }
            else
                for (const auto &map : joint_idmap_)
                    home_positions_[map.first] = 0.0;

            if (hwconfig["real_state_enable"])
            {
                real_state_enable_ = hwconfig["real_state_enable"].as<bool>();
                LOG_INFO("> Real state enable: " << real_state_enable_);
            }
            else
            {
                real_state_enable_ = false;
                LOG_WARN("> Real state enable not found, using default: " << real_state_enable_);
            }
            if (hwconfig["real_state_period"])
            {
                real_state_period_ = hwconfig["real_state_period"].as<double>();
                LOG_INFO("> Real state period: " << real_state_period_);
            }
            else
            {
                real_state_period_ = DEFAULT_REAL_STATE_PERIOD;
                LOG_WARN("> Real state period not found, using default: " << real_state_period_);
            }
            if (hwconfig["real_state_timeout"])
            {
                real_state_timeout_ = hwconfig["real_state_timeout"].as<double>();
                LOG_INFO("> Real state timeout: " << real_state_timeout_);
            }
            else
            {
                real_state_timeout_ = DEFAULT_REAL_STATE_TIMEOUT;
                LOG_WARN("> Real state timeout not found, using default: " << real_state_timeout_);
            }
            if (hwconfig["action_group_topic"])
            {
                action_group_topic_ = hwconfig["action_group_topic"].as<std::string>();
                LOG_INFO("> Action group topic: " << action_group_topic_);
            }
            else
            {
                action_group_topic_ = "~/action_group";
                LOG_WARN("> Action group topic not found, using default: " << action_group_topic_);
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Failed to load configuration file: " << config_file_path_ << " with exception: " << e.what());
            return CallbackReturn::ERROR;
        }
        LOG_INFO("HwsystemAg initialized successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HwsystemAg::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        try
        {
            LOG_INFO("Loading driver device ...");
            pluginlib::ClassLoader<idevice::Servo> loader("idevice", "idevice::Servo");
            servo_ = loader.createSharedInstance("lsc_servo::Lsc_servo");

            if (!servo_->connect())
            {
                LOG_ERROR("Failed to connect to device");
                return CallbackReturn::ERROR;
            }

            rclcpp::NodeOptions options;
            options.automatically_declare_parameters_from_overrides(true);
            node_ = std::make_shared<rclcpp::Node>("hwsystem", "backend", options);
            action_group_sub_ = node_->create_subscription<group_action::msg::GroupAction>(action_group_topic_, rclcpp::QoS(10), std::bind(&HwsystemAg::action_group_callback, this, std::placeholders::_1));

            LOG_INFO("Driver device connected successfully");
        }
        catch (const pluginlib::PluginlibException &e)
        {
            LOG_ERROR("Failed to load the driver device with exception: " << e.what());
            return CallbackReturn::ERROR;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Failed to connect to device with exception: " << e.what());
            return CallbackReturn::ERROR;
        }

        LOG_INFO("HwsystemAg configured successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HwsystemAg::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        for (const auto &joint : joint_idmap_)
        {
            joint_state_positions_[joint.first] = home_positions_.at(joint.first);
            LOG_INFO("Set home position joint [" << joint.first << "] : " << joint_state_positions_[joint.first]);
        }
        LOG_INFO("HwsystemAg activated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HwsystemAg::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        if (servo_)
        {
            servo_->stopActionGroup();
        }
        LOG_INFO("HwsystemAg deactivated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HwsystemAg::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        if (servo_ && servo_->isConnected())
        {
            servo_->disconnect();
            LOG_INFO("Driver device disconnected during cleanup");
        }
        joint_state_positions_.clear();
        home_positions_.clear();
        joint_idmap_.clear();
        servo_.reset();

        LOG_INFO("HwsystemAg cleaned up successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HwsystemAg::on_error([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_ERROR("HwsystemAg encountered an error");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HwsystemAg::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("HwsystemAg shutdown successfully");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type HwsystemAg::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if (!servo_)
        {
            LOG_ERROR("Driver device not loaded");
            return hardware_interface::return_type::ERROR;
        }

        if (action_group_running_)
            return hardware_interface::return_type::OK;

        try
        {
            std::vector<uint8_t> servo_ids;
            servo_ids.reserve(joint_idmap_.size());
            for (const auto &map : joint_idmap_)
                servo_ids.push_back(map.second);
            std::map<uint8_t, double> state_servos = servo_->stateServo(servo_ids, (uint16_t)real_state_timeout_);
            for (const auto &map : joint_idmap_)
            {
                auto servo_state = state_servos.find(map.second);
                if (servo_state != state_servos.end())
                {
                    double real_position = servo_state->second;

                    joint_state_positions_.at(map.first) = real_position;
                    LOG_INFO("> Read joint: " << map.first
                                              << " | Position: " << joint_state_positions_.at(map.first)
                                              << " | Timeout: " << real_state_timeout_);
                }
            }
        }
        catch (const std::out_of_range &e)
        {
            LOG_ERROR("Out of range error during read: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Read failed with exception: " << e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type HwsystemAg::write([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if (!servo_)
        {
            LOG_ERROR("Driver device not loaded");
            return hardware_interface::return_type::ERROR;
        }

        if (node_)
            rclcpp::spin_some(node_);

        if (!action_group_running_ && action_group_)
        {
            if (servo_->runActionGroup(action_group_->group_id, action_group_->repetitions))
            {
                action_group_running_ = true;
                LOG_INFO("Action group [" << (int)action_group_->group_id << "] started with " << (action_group_->repetitions == 0 ? "infinite" : std::to_string(action_group_->repetitions)) << " repetitions");
            }
            else
            {
                LOG_WARN("Action group [" << (int)action_group_->group_id << "] failed to start");
                action_group_.reset();
            }
            return hardware_interface::return_type::OK;
        }

        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> HwsystemAg::on_export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
        for (const auto &joint : joint_state_positions_)
        {
            state_interfaces.emplace_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint.first,
                    hardware_interface::HW_IF_POSITION,
                    &joint_state_positions_.at(joint.first)));
        }
        LOG_INFO("Exported (" << state_interfaces.size() << ") state interfaces");
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> HwsystemAg::on_export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
        for (const auto &joint : joint_command_positions_)
        {
            command_interfaces.emplace_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint.first,
                    hardware_interface::HW_IF_POSITION,
                    &joint_command_positions_.at(joint.first)));
        }
        LOG_INFO("Exported (" << command_interfaces.size() << ") command interfaces");
        return command_interfaces;
    }

    void HwsystemAg::action_group_callback(const group_action::msg::GroupAction::SharedPtr msg)
    {
        if (action_group_running_ && msg->group_id != 0)
        {
            LOG_WARN("Action group already running");
            return;
        }

        if (msg->group_id == 0)
        {
            if (action_group_running_ && servo_->stopActionGroup())
            {
                action_group_running_ = false;
                action_group_.reset();
                LOG_INFO("Action group stopped");
            }
            return;
        }

        action_group_ = msg;
        LOG_INFO("Action group ID: " << (int)msg->group_id << ", Repetitions: " << (msg->repetitions == 0 ? "infinite" : std::to_string(msg->repetitions)));
    }

} // namespace hwsystem_ag
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hwsystem_ag::HwsystemAg, hardware_interface::SystemInterface)
