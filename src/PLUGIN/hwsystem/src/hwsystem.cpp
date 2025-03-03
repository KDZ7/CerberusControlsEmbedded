#include <cmath>
#include "hwsystem/hwsystem.hpp"
#include "yaml-cpp/yaml.h"
#include "pluginlib/class_loader.hpp"

#define DEFAULT_MOVETIME 25.0
#define DEFAULT_REAL_STATE_TIMEOUT 1000.0
#define DEFAULT_REAL_STATE_PERIOD 1000.0
#define DEFAULT_REAL_STATE_TOLERANCE 1e-2
#define DEFAULT_MIN_POSITION -0.785398163
#define DEFAULT_MAX_POSITION 0.785398163397

namespace hwsystem
{
    CallbackReturn Hwsystem::on_init(const hardware_interface::HardwareInfo &info)
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
            if (hwconfig["movetime"])
            {
                movetime_ = hwconfig["movetime"].as<double>();
                LOG_INFO("> State move time: " << movetime_);
            }
            else
            {
                movetime_ = DEFAULT_MOVETIME;
                LOG_WARN("> Movetime not found, using default: " << movetime_);
            }
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
            if (hwconfig["real_state_tolerance"])
            {
                real_state_tolerance_ = hwconfig["real_state_tolerance"].as<double>();
                LOG_INFO("> Real state tolerance: " << real_state_tolerance_);
            }
            else
            {
                real_state_tolerance_ = DEFAULT_REAL_STATE_TOLERANCE;
                LOG_WARN("> Real state tolerance not found, using default: " << real_state_tolerance_);
            }
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Failed to load configuration file: " << config_file_path_ << " with exception: " << e.what());
            return CallbackReturn::ERROR;
        }
        LOG_INFO("Hwsystem initialized successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
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

        LOG_INFO("Hwsystem configured successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        for (const auto &joint : joint_idmap_)
        {
            joint_state_positions_[joint.first] = home_positions_.at(joint.first);
            LOG_INFO("Set home position joint [" << joint.first << "] : " << joint_state_positions_[joint.first]);
        }
        LOG_INFO("Hwsystem activated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        if (servo_)
        {
            std::vector<std::tuple<uint8_t, double>> servos;
            for (const auto &joint : joint_idmap_)
            {
                servos.push_back({joint.second, home_positions_.at(joint.first)});
                LOG_INFO("Go to home position joint: " << joint.first << " Position: " << home_positions_.at(joint.first));
            }
            if (!servos.empty())
                servo_->moveServo(servos, (uint16_t)movetime_);
        }
        LOG_INFO("Hwsystem deactivated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        if (servo_ && servo_->isConnected())
        {
            servo_->disconnect();
            LOG_INFO("Driver device disconnected during cleanup");
        }
        joint_state_positions_.clear();
        joint_command_positions_.clear();
        joint_max_positions_.clear();
        joint_min_positions_.clear();
        home_positions_.clear();
        joint_idmap_.clear();
        servo_.reset();

        LOG_INFO("Hwsystem cleaned up successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_error([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_ERROR("Hwsystem encountered an error");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Hwsystem::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("Hwsystem shutdown successfully");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Hwsystem::read([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if (!servo_)
        {
            LOG_ERROR("Driver device not loaded");
            return hardware_interface::return_type::ERROR;
        }

        for (const auto &joint : joint_command_positions_)
            joint_state_positions_[joint.first] = joint_command_positions_[joint.first];

        if (!real_state_enable_)
            return hardware_interface::return_type::OK;

        static rclcpp::Time last_read_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
        if (time - last_read_time < rclcpp::Duration::from_seconds(real_state_period_ / 1000.0))
            return hardware_interface::return_type::OK;

        last_read_time = time;

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
                    double command_position = joint_command_positions_.at(map.first);
                    if (std::abs(real_position - command_position) <= real_state_tolerance_)
                    {
                        joint_state_positions_.at(map.first) = real_position;
                        LOG_INFO("> Read joint: " << map.first
                                                  << " | Position: " << joint_state_positions_.at(map.first)
                                                  << " | Timeout: " << real_state_timeout_);
                    }
                    else
                        LOG_WARN("> Read Joint: " << map.first
                                                  << " | Out of tolerance: " << std::abs(real_position - command_position)
                                                  << " | Tolerance: " << real_state_tolerance_
                                                  << " | Real: " << real_position
                                                  << " | Command: " << command_position);
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
    hardware_interface::return_type Hwsystem::write([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if (!servo_)
        {
            LOG_ERROR("Driver device not loaded");
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            std::vector<std::tuple<uint8_t, double>> servo_commands;
            servo_commands.reserve(joint_command_positions_.size());
            for (const auto &joint : joint_command_positions_)
            {
                auto servo_id = joint_idmap_.find(joint.first);
                if (servo_id != joint_idmap_.end())
                {
                    double target_position = joint_command_positions_.at(joint.first);
                    target_position = std::max(
                        joint_min_positions_.at(joint.first),
                        std::min(joint_max_positions_.at(joint.first), target_position));
                    servo_commands.push_back({servo_id->second, target_position});
                    LOG_INFO("> Write joint: " << joint.first
                                               << " | Position: " << target_position
                                               << " | Time: " << movetime_);
                }
            }
            if (!servo_commands.empty())
                servo_->moveServo(servo_commands, (uint16_t)movetime_);
        }
        catch (const std::out_of_range &e)
        {
            LOG_ERROR("Out of range error during write: " << e.what());
            return hardware_interface::return_type::ERROR;
        }
        catch (const std::exception &e)
        {
            LOG_ERROR("Write failed with exception: " << e.what());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> Hwsystem::on_export_state_interfaces()
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

    std::vector<hardware_interface::CommandInterface::SharedPtr> Hwsystem::on_export_command_interfaces()
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
} // namespace hwsystem

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hwsystem::Hwsystem, hardware_interface::SystemInterface)