#ifndef __CSYSTEM_HPP__
#define __CSYSTEM_HPP__

#include "log/log.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "control_toolbox/pid.hpp"
#include "controller_interface/controller_interface.hpp"
#include "csystem/visibility_control.h"

namespace csystem
{
  class Csystem : public controller_interface::ControllerInterface
  {
  public:
    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    std::vector<std::string> joints_;
    std::vector<std::string> state_interfaces_types_;
    std::vector<std::string> command_interfaces_types_;
    std::string topic_pub_;
    std::string topic_sub_;

    std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> rt_joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    realtime_tools::RealtimeBuffer<std::vector<double>> rt_joint_position_buffers_;
    realtime_tools::RealtimeBuffer<std::vector<double>> rt_joint_velocity_buffers_;
    realtime_tools::RealtimeBuffer<std::vector<double>> rt_joint_effort_buffers_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_state_interfaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> velocity_state_interfaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> effort_state_interfaces_;

    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> position_command_interfaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> velocity_command_interfaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> effort_command_interfaces_;

    std::vector<double> joint_offsets_;
    double kp_, ki_, kd_, i_clamp_;
    double dthreshold_;
    double dcorrection_;
    double smoothing_factor_;

    std::vector<double> previous_errors_;
    std::vector<control_toolbox::Pid> pid_controllers_;
    std::string frame_id_;
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  };

} // namespace csystem

#endif // __CSYSTEM_HPP__