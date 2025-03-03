#ifndef __HWSYSTEM_HPP__
#define __HWSYSTEM_HPP__

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hwsystem/visibility_control.h"
#include "idevice/idevice.hpp"

namespace hwsystem
{

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  class Hwsystem : public hardware_interface::SystemInterface
  {
  public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  private:
    std::map<std::string, double> joint_state_positions_;
    std::map<std::string, double> joint_command_positions_;
    std::map<std::string, double> joint_max_positions_;
    std::map<std::string, double> joint_min_positions_;
    std::map<std::string, double> home_positions_;
    double movetime_;
    bool real_state_enable_;
    double real_state_period_;
    double real_state_timeout_;
    double real_state_tolerance_;

    std::string config_file_path_;
    std::map<std::string, uint8_t> joint_idmap_;
    std::shared_ptr<idevice::Servo> servo_;
  };

} // namespace hwsystem

#endif // __HWSYSTEM_HPP__
