#include "csystem/csystem.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace csystem
{
    controller_interface::CallbackReturn Csystem::on_init()
    {
        joints_ = auto_declare<std::vector<std::string>>("joints", joints_);
        state_interfaces_types_ = auto_declare<std::vector<std::string>>("state_interfaces", state_interfaces_types_);
        command_interfaces_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interfaces_types_);
        topic_sub_ = auto_declare<std::string>("topic_sub", "in");
        topic_pub_ = auto_declare<std::string>("topic_pub", "out");
        joint_offsets_ = auto_declare<std::vector<double>>("joint_offsets", std::vector<double>(joints_.size(), 0.0));

        kp_ = auto_declare<double>("kp", 1.0);
        ki_ = auto_declare<double>("ki", 0.0);
        kd_ = auto_declare<double>("kd", 0.0);
        i_clamp_ = auto_declare<double>("i_clamp", 0.0);
        dthreshold_ = auto_declare<double>("dthreshold", 0.005);
        dcorrection_ = auto_declare<double>("dcorrection", 0.01);
        smoothing_factor_ = auto_declare<double>("smoothing_factor", 0.9);

        LOG_INFO("Csystem initialized successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Csystem::on_configure([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        if (joints_.empty())
        {
            LOG_ERROR("No joints specified");
            return controller_interface::CallbackReturn::ERROR;
        }
        rt_joint_state_pub_ = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(this->get_node()->create_publisher<sensor_msgs::msg::JointState>(topic_pub_, rclcpp::QoS(10)));
        rt_joint_state_pub_->msg_.name = joints_;
        rt_joint_state_pub_->msg_.position.resize(joints_.size(), 0.0);
        rt_joint_state_pub_->msg_.velocity.resize(joints_.size(), 0.0);
        rt_joint_state_pub_->msg_.effort.resize(joints_.size(), 0.0);
        joint_states_sub_ = this->get_node()->create_subscription<sensor_msgs::msg::JointState>(topic_sub_, rclcpp::QoS(10), std::bind(&Csystem::joint_states_callback, this, std::placeholders::_1));

        if (joint_offsets_.size() != joints_.size())
        {
            LOG_WARN("Joint offsets size does not match the number of joints. Resizing automatically");
            joint_offsets_.resize(joints_.size(), 0.0);
        }

        previous_errors_.resize(joints_.size(), 0.0);
        pid_controllers_.resize(joints_.size(), control_toolbox::Pid(kp_, ki_, kd_, i_clamp_, -i_clamp_));
        for (size_t i = 0; i < joints_.size(); ++i)
            pid_controllers_[i].initialize(kp_, ki_, kd_, i_clamp_, -i_clamp_);
        LOG_INFO("Csystem configured successfully");

        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn Csystem::on_activate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        rt_joint_position_buffers_.writeFromNonRT(joint_offsets_);
        rt_joint_velocity_buffers_.writeFromNonRT(std::vector<double>(joints_.size(), 0.0));
        rt_joint_effort_buffers_.writeFromNonRT(std::vector<double>(joints_.size(), 0.0));

        position_state_interfaces_.clear();
        velocity_state_interfaces_.clear();
        effort_state_interfaces_.clear();
        position_command_interfaces_.clear();
        velocity_command_interfaces_.clear();
        effort_command_interfaces_.clear();

        for (auto &interface : state_interfaces_)
        {
            if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
                position_state_interfaces_.push_back(interface);
            else if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                velocity_state_interfaces_.push_back(interface);
            else if (interface.get_interface_name() == hardware_interface::HW_IF_EFFORT)
                effort_state_interfaces_.push_back(interface);
        }

        for (auto &interface : command_interfaces_)
        {
            if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
                position_command_interfaces_.push_back(interface);
            else if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
                velocity_command_interfaces_.push_back(interface);
            else if (interface.get_interface_name() == hardware_interface::HW_IF_EFFORT)
                effort_command_interfaces_.push_back(interface);
        }
        LOG_INFO("Cystem details: " << std::endl
                                    << "> Joints size: " << joints_.size() << std::endl
                                    << "> State Interfaces: " << state_interfaces_types_.size() << std::endl
                                    << "> Command Interfaces: " << command_interfaces_types_.size() << std::endl
                                    << "> Topic Publisher: " << topic_pub_ << std::endl
                                    << "> Topic Subscriber: " << topic_sub_ << std::endl
                                    << "> Joint Offsets: " << joint_offsets_.size() << std::endl
                                    << "> Kp: " << kp_ << std::endl
                                    << "> Ki: " << ki_ << std::endl
                                    << "> Kd: " << kd_ << std::endl
                                    << "> I Clamp: " << i_clamp_ << std::endl
                                    << "> D Threshold: " << dthreshold_ << std::endl
                                    << "> D Correction: " << dcorrection_ << std::endl
                                    << "> Smoothing Factor: " << smoothing_factor_ << std::endl);

        LOG_INFO("Csystem activated successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn Csystem::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("Csystem deactivated successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::CallbackReturn Csystem::on_cleanup([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        joints_.clear();
        state_interfaces_types_.clear();
        command_interfaces_types_.clear();
        topic_pub_.clear();
        topic_sub_.clear();
        rt_joint_state_pub_.reset();
        joint_states_sub_.reset();
        rt_joint_position_buffers_.reset();
        rt_joint_velocity_buffers_.reset();
        rt_joint_effort_buffers_.reset();
        position_state_interfaces_.clear();
        velocity_state_interfaces_.clear();
        effort_state_interfaces_.clear();
        position_command_interfaces_.clear();
        velocity_command_interfaces_.clear();
        effort_command_interfaces_.clear();
        joint_offsets_.clear();
        previous_errors_.clear();
        pid_controllers_.clear();
        frame_id_.clear();

        LOG_INFO("Csystem cleaned up successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Csystem::on_error([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_ERROR("Csystem encountered an error");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Csystem::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State &previous_state)
    {
        LOG_INFO("Csystem shutdown successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::InterfaceConfiguration Csystem::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // Claim the interfaces specified in the config file [YAML]
        std::vector<std::string> state_interfaces;
        if (this->get_node()->has_parameter("state_interfaces"))
            this->get_node()->get_parameter("state_interfaces", state_interfaces);
        if (!state_interfaces.empty())
            for (const auto &joint : joints_)
                for (const auto &state_interface : state_interfaces)
                    config.names.push_back(joint + "/" + state_interface);
        return config;
    }
    controller_interface::InterfaceConfiguration Csystem::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // Claim the interfaces specified in the config file [YAML]
        std::vector<std::string> command_interfaces;
        if (this->get_node()->has_parameter("command_interfaces"))
            this->get_node()->get_parameter("command_interfaces", command_interfaces);
        if (!command_interfaces.empty())
            for (const auto &joint : joints_)
                for (const auto &command_interface : command_interfaces)
                    config.names.push_back(joint + "/" + command_interface);

        return config;
    }
    controller_interface::return_type Csystem::update([[maybe_unused]] const rclcpp::Time &time, [[maybe_unused]] const rclcpp::Duration &period)
    {
        if (position_state_interfaces_.empty() || position_command_interfaces_.empty())
        {
            LOG_WARN("Position state or command interfaces not initialized yet, ignoring update");
            return controller_interface::return_type::OK;
        }
        if (!rt_joint_position_buffers_.readFromRT() || !rt_joint_velocity_buffers_.readFromRT() || !rt_joint_effort_buffers_.readFromRT())
        {
            LOG_WARN("RT buffers not initialized yet, ignoring update");
            return controller_interface::return_type::OK;
        }
        std::vector<double> positions = *rt_joint_position_buffers_.readFromRT();
        std::vector<double> velocities = *rt_joint_velocity_buffers_.readFromRT();
        std::vector<double> efforts = *rt_joint_effort_buffers_.readFromRT();
        for (size_t i = 0; i < joints_.size(); ++i)
        {
            if (i < position_command_interfaces_.size() && i < position_state_interfaces_.size())
            {
                double current_position = position_state_interfaces_[i].get().get_optional().value_or(0.0);
                LOG_INFO("> Joint: " << joints_[i]
                                     << " | Current: " << current_position
                                     << " | Target: " << positions[i]);
                double target_position = positions[i];
                if (std::isnan(current_position) || std::isinf(current_position))
                {
                    LOG_WARN("Invalid current position (NaN/Inf) for joint [" << joints_[i] << "]");
                    continue;
                }
                if (std::isnan(target_position) || std::isinf(target_position))
                {
                    LOG_WARN("Invalid target position (NaN/Inf) for joint [" << joints_[i] << "]");
                    continue;
                }
                double error = target_position - current_position;
                double filtered_error = smoothing_factor_ * previous_errors_[i] + (1.0 - smoothing_factor_) * error;
                double correction = pid_controllers_[i].compute_command(filtered_error, period);
                correction = std::max(-dcorrection_, std::min(dcorrection_, correction));
                double new_position = current_position + correction;
                previous_errors_[i] = filtered_error;
                if (std::abs(target_position - current_position) > dthreshold_)
                {
                    LOG_INFO("> Joint: " << joints_[i]
                                         << " | Current: " << current_position
                                         << " | Target: " << positions[i]
                                         << " | Error: " << error
                                         << " | Filtered Error: " << filtered_error
                                         << " | Correction: " << correction
                                         << " | New Position: " << new_position);
                }
                [[maybe_unused]] bool _ = position_command_interfaces_[i].get().set_value(new_position);
            }
            if (i < velocity_command_interfaces_.size() && i < velocity_state_interfaces_.size())
            {
                [[maybe_unused]] bool _ = velocity_command_interfaces_[i].get().set_value(velocities[i]);
            }
            if (i < effort_command_interfaces_.size() && i < effort_state_interfaces_.size())
            {
                [[maybe_unused]] bool _ = effort_command_interfaces_[i].get().set_value(efforts[i]);
            }
        }
        if (rt_joint_state_pub_ && rt_joint_state_pub_->trylock())
        {
            auto &msg = rt_joint_state_pub_->msg_;
            msg.header.stamp = time;
            msg.header.frame_id = frame_id_;
            msg.name = joints_;
            for (size_t i = 0; i < joints_.size(); ++i)
            {
                if (i < position_state_interfaces_.size())
                    msg.position[i] = position_state_interfaces_[i].get().get_optional().value_or(0.0);
                if (i < velocity_state_interfaces_.size())
                    msg.velocity[i] = velocity_state_interfaces_[i].get().get_optional().value_or(0.0);
                if (i < effort_state_interfaces_.size())
                    msg.effort[i] = effort_state_interfaces_[i].get().get_optional().value_or(0.0);
            }
            rt_joint_state_pub_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }
    void Csystem::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->name.empty())
        {
            LOG_WARN("Received empty joint command message");
            return;
        }
        if (position_state_interfaces_.empty())
        {
            LOG_WARN("Position state interfaces not initialized yet, ignoring command message");
            return;
        }

        if (!msg->header.frame_id.empty())
            frame_id_ = msg->header.frame_id;

        std::vector<double> positions = *rt_joint_position_buffers_.readFromRT();
        std::vector<double> velocities = *rt_joint_velocity_buffers_.readFromRT();
        std::vector<double> efforts = *rt_joint_effort_buffers_.readFromRT();

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            auto it = std::find(joints_.begin(), joints_.end(), msg->name[i]);
            if (it != joints_.end())
            {
                size_t idx = std::distance(joints_.begin(), it);
                if (idx < positions.size() && i < msg->position.size())
                {
                    double target_position = msg->position[i];
                    if (std::isnan(target_position) || std::isinf(target_position))
                    {
                        LOG_WARN("Received an invalid target position (NaN/Inf) for joint [" << msg->name[i] << "]");
                        continue;
                    }
                    double current_position = position_state_interfaces_[idx].get().get_optional().value_or(0.0);
                    if (std::abs(target_position - current_position) > dthreshold_)
                        positions[idx] = target_position;
                    else
                        continue;
                }
                if (idx < velocities.size() && i < msg->velocity.size())
                    velocities[idx] = msg->velocity[i];
                if (idx < efforts.size() && i < msg->effort.size())
                    efforts[idx] = msg->effort[i];
            }
        }
        rt_joint_position_buffers_.writeFromNonRT(positions);
        rt_joint_velocity_buffers_.writeFromNonRT(velocities);
        rt_joint_effort_buffers_.writeFromNonRT(efforts);
    }
} // namespace csystem
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(csystem::Csystem, controller_interface::ControllerInterface)