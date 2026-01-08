#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <cstdint> // for serial comm.

namespace opencr_diffbot_hw
{

class OpenCRSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OpenCRSystem)

  // ros2_control lifecycle
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // state/command interface export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // periodic I/O (controller_manager loop에서 호출)
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ----- Parameters (URDF/ros2_control xacro에서 넘어옴) -----
  std::string device_{"/dev/ttyACM0"};
  int baud_{115200};

  // ----- Joint names (URDF와 일치해야 함) -----
  std::string left_joint_name_;
  std::string right_joint_name_;

  // ----- State (position[rad], velocity[rad/s]) -----
  std::vector<double> pos_;   // size 2
  std::vector<double> vel_;   // size 2


  // ----- IMU State -----
  // 단위: rad/s, m/s^2
  double imu_gx_{0.0}, imu_gy_{0.0}, imu_gz_{0.0};
  double imu_ax_{0.0}, imu_ay_{0.0}, imu_az_{0.0};
  // ---- IMU orientation (quaternion) ----
  double imu_qx_{0.0}, imu_qy_{0.0}, imu_qz_{0.0}, imu_qw_{1.0};

  // --- yaw integration (for RViz IMU orientation visualization) ---
  bool imu_has_last_ms_{false};
  long imu_last_ms_{0};
  double imu_yaw_int_{0.0};  // integrated yaw [rad]

  // ----- Command (velocity[rad/s]) -----
  std::vector<double> cmd_vel_;  // size 2


  // serial comm part for Linux termios
  int fd_{-1};
  bool serial_ok_{false};

  // Rx line buffer until \n
  std::string rx_buf_;

  // internal helper
  bool open_serial_();
  void close_serial_();
  bool read_line_(std::string & line_out);
  bool parse_state_line_(const std::string & line);
  bool parse_imu_line_(const std::string & line);
  bool write_cmd_(double w1_radps, double w2_radps);



};

}  // namespace opencr_diffbot_hw
