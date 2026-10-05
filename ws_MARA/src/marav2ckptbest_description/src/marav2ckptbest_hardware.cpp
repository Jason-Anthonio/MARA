#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace marav2ckptbest_hardware
{

class StepHardware final : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto joint_count = info.joints.size();
    positions_.assign(joint_count, 0.0);
    velocities_.assign(joint_count, 0.0);
    efforts_.assign(joint_count, 0.0);
    position_commands_.assign(joint_count, 0.0);
    velocity_commands_.assign(joint_count, 0.0);
    last_position_commands_.assign(joint_count, 0.0);

    for (std::size_t index = 0; index < info.joints.size(); ++index) {
      const auto & joint = info.joints[index];
      if (joint.command_interfaces.size() > 2) {
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.name == "Slider_10") {
        slider_10_index_ = index;
      } else if (joint.name == "Slider_11") {
        slider_11_index_ = index;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> interfaces;
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      interfaces.emplace_back(info_.joints[index].name, "position", &positions_[index]);
      interfaces.emplace_back(info_.joints[index].name, "velocity", &velocities_[index]);
      interfaces.emplace_back(info_.joints[index].name, "effort", &efforts_[index]);
    }
    return interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      for (const auto & command_interface : info_.joints[index].command_interfaces) {
        if (command_interface.name == hardware_interface::HW_IF_POSITION) {
          interfaces.emplace_back(info_.joints[index].name, command_interface.name,
            &position_commands_[index]);
        } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
          interfaces.emplace_back(info_.joints[index].name, command_interface.name,
            &velocity_commands_[index]);
        }
      }
    }
    return interfaces;
  }

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    last_position_commands_ = position_commands_;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    const double seconds = std::max(0.0, period.seconds());
    for (std::size_t index = 0; index < positions_.size(); ++index) {
      if (index == slider_11_index_) {
        continue;
      }

      const double position_delta = position_commands_[index] - last_position_commands_[index];
      if (std::abs(position_delta) > command_epsilon_) {
        positions_[index] = position_commands_[index];
        last_position_commands_[index] = position_commands_[index];
      } else {
        positions_[index] += velocity_commands_[index] * seconds;
      }
      velocities_[index] = velocity_commands_[index];
    }

    if (slider_10_index_ != invalid_index && slider_11_index_ != invalid_index) {
      positions_[slider_11_index_] = -positions_[slider_10_index_];
      velocities_[slider_11_index_] = -velocities_[slider_10_index_];
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return hardware_interface::return_type::OK;
  }

private:
  static constexpr std::size_t invalid_index = std::numeric_limits<std::size_t>::max();
  static constexpr double command_epsilon_ = 1e-9;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  std::vector<double> position_commands_;
  std::vector<double> velocity_commands_;
  std::vector<double> last_position_commands_;
  std::size_t slider_10_index_{invalid_index};
  std::size_t slider_11_index_{invalid_index};
};

}  // namespace marav2ckptbest_hardware

PLUGINLIB_EXPORT_CLASS(
  marav2ckptbest_hardware::StepHardware, hardware_interface::SystemInterface)