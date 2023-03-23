#ifndef OMRON1S_HARDWARE__OMRON1S_HARDWARE_HPP_
#define OMRON1S_HARDWARE__OMRON1S_HARDWARE_HPP_

#include <map>
#include <vector>

#include "rclcpp/macros.hpp"
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <string.h>
#include <inttypes.h>
#include "ethercat.h"

#define EC_TIMEOUTMON 500
#define ENCODER_RESOLUTION 8388608
#define PI 3.14159265359

using hardware_interface::return_type;

namespace omron1s_hardware
{
  struct JointValue
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
  };

  struct Joint
  {
    JointValue state{};
    JointValue command{};
  };

  enum class ControlMode
  {
    Position,
    Velocity,
    Torque,
    Currrent,
    ExtendedPosition,
    MultiTurn,
    CurrentBasedPosition,
    PWM,
  };

  typedef struct PACKED
  {
    uint16 controlword;
    int32 target_position;
    uint16 touch_probe_function;
    uint32 physical_outputs;
  } output_R88Dt;

  typedef struct PACKED
  {
    uint16 error_code;
    uint16 statusword;
    int32 position_actual_value;
    int16 torque_actual_value;
    int32 following_error_actual_value;
    uint16 touch_probe_status;
    int32 touch_probe1_positive_edge;
    int32 touch_probe2_positive_edge;
    uint32 digital_inputs;
  } input_R88Dt;

  class Omron1SHardware : public hardware_interface::SystemInterface
  {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(Omron1SHardware)

    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    return_type read() override;

    return_type write() override;

  private:
    return_type reset_command();

    std::vector<Joint> joints_;
    std::vector<uint8_t> joint_ids_;
    bool torque_enabled_{false};
    bool use_dummy_{false};


    char IOmap[80];
    OSAL_THREAD_HANDLE thread1;
    int expectedWKC;
    boolean needlf;
    volatile int wkc;
    boolean inOP;
    uint8 currentgroup;
    output_R88Dt *output_R88D;
    input_R88Dt *input_R88D;
    // int R88Dsetup(uint16 slave);
    // OSAL_THREAD_FUNC ecatcheck(void *ptr);
  };
} // namespace omron1s_hardware

#endif // OMRON1S_HARDWARE__OMRON1S_HARDWARE_HPP_