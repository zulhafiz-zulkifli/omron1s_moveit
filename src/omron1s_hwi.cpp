#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>

#include "omron1s_hwi/omron1s_hwi.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace omron1s_hardware
{

  CallbackReturn
  Omron1SHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(Omron1SHardware), "configure");
    if (hardware_interface::SystemInterface::on_init(info) !=
        CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    joints_.resize(info_.joints.size(), Joint());
    joint_ids_.resize(info_.joints.size(), 0);

    for (uint i = 0; i < info_.joints.size(); i++)
    {
      joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
      joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
      joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
      joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "joint_id %d: %d", i,
                  joint_ids_[i]);
    }

    if (info_.hardware_parameters.find("use_dummy") !=
            info_.hardware_parameters.end() &&
        info_.hardware_parameters.at("use_dummy") == "true")
    {
      use_dummy_ = true;
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "dummy mode");
      return CallbackReturn::SUCCESS;
    }

    RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "SOEM (Simple Open EtherCAT Master)");
    /* create thread to handle slave error handling in OP */
    osal_thread_create(&thread1, 128000, &ecatcheck, (void *)&ctime);

    needlf = FALSE;
    inOP = FALSE;

    /////////////////////////////////////////////////////////////////////////////////////////
    /* initialise SOEM, bind socket to ifname */
    auto interface_name = info_.hardware_parameters.at("interface_name");
    RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "interface_name: %s", interface_name.c_str());
    if (ec_init(interface_name))
    {
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "ec_init on %s succeeded.", interface_name.c_str());
      /* find and auto-config slaves */

      if (ec_config_init(FALSE) > 0)
      {
        RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "%d slaves found and configured.\n", ec_slavecount);
        if (ec_slavecount == info_.joints.size())
        {
          for (int i = 1; i <= ec_slavecount; i++)
          {
            if (strcmp(ec_slave[i].name, "R88D-1SN01H-ECT") || strcmp(ec_slave[i].name, "R88D-1SN02H-ECT"))
            {
              printf("Found %s at position %d\n", ec_slave[i].name, i);
              /* link slave specific setup to preop->safeop hook */
              ec_slave[i].PO2SOconfig = R88Dsetup;
            }
          }
          
          ec_config_map(&IOmap);
          ec_configdc();

          printf("Slaves mapped, state to SAFE_OP.\n");
          /* wait for all slaves to reach SAFE_OP state */
          ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
          printf("Request operational state for all slaves\n");
          expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
          printf("Calculated workcounter %d\n", expectedWKC);
          ec_slave[0].state = EC_STATE_OPERATIONAL;
          /* send one valid process data to make outputs in slaves happy*/
          ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          /* request OP state for all slaves */
          ec_writestate(0);
          int chk = 200;
          /* wait for all slaves to reach OP state */
          do
          {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
          } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

          if (ec_slave[0].state == EC_STATE_OPERATIONAL)
          {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;

            /* cyclic loop */
            int i = 1;
            while (TRUE)
            {
              for (int s = 1; s <= ec_slavecount; s++)
              {
                for (int s = 1; s <= ec_slavecount; s++)
                {
                  input_R88D = (input_R88Dt *)ec_slave[s].inputs;

                  printf("slave %d pos : %10d     ", s, input_R88D->position_actual_value);
                }
                printf("\n");

                output_R88D = (output_R88Dt *)ec_slave[s].outputs;
                if (output_R88D->controlword == 0)
                {
                  output_R88D->controlword = 6;
                }
                else if (output_R88D->controlword == 6)
                {
                  output_R88D->controlword = 7;
                }
                else if (output_R88D->controlword == 7)
                {
                  output_R88D->controlword = 15;
                }
              }

              ec_send_processdata();
              wkc = ec_receive_processdata(EC_TIMEOUTRET);

              if (wkc >= expectedWKC)
              {
                needlf = TRUE;
              }
              osal_usleep(1000);
              i++;
              if (i == 2000)
              {
                printf("_______________________________________________________CONTROL BREAK!!!!\n");
                break;
              }
            }

            while (TRUE)
            {
              for (int s = 1; s <= ec_slavecount; s++)
              {
                for (int s = 1; s <= ec_slavecount; s++)
                {
                  input_R88D = (input_R88Dt *)ec_slave[s].inputs;

                  printf("slave %d pos : %10d     ", s, input_R88D->position_actual_value);
                }
                printf("\n");

                output_R88D = (output_R88Dt *)ec_slave[s].outputs;
                if (output_R88D->controlword == 15)
                {
                  output_R88D->controlword = 7;
                }
                else if (output_R88D->controlword == 7)
                {
                  output_R88D->controlword = 6;
                }
                else if (output_R88D->controlword == 6)
                {
                  output_R88D->controlword = 0;
                }
              }

              ec_send_processdata();
              wkc = ec_receive_processdata(EC_TIMEOUTRET);

              if (wkc >= expectedWKC)
              {
                needlf = TRUE;
              }
              osal_usleep(1000);
              break;
            }

            inOP = FALSE;
          }
          else
          {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            for (int i = 1; i <= ec_slavecount; i++)
            {
              if (ec_slave[i].state != EC_STATE_OPERATIONAL)
              {
                printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                       i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
              }
            }
          }
          printf("\nRequest init state for all slaves\n");
          ec_slave[0].state = EC_STATE_INIT;
          /* request INIT state for all slaves */
          ec_writestate(0);
        }
        else
        {
          RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "More or less than %d slaves found!", info_.joints.size());
          ec_close();
          return CallbackReturn::ERROR;
        }
      }
      else
      {
        RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "No slaves found!");
        ec_close();
        return CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "No socket connection on %s", interface_name.c_str());
      return CallbackReturn::ERROR;
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    auto baud_rate = std::stoi(info_.hardware_parameters.at("baud_rate"));
    const char *log = nullptr;

    RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "usb_port: %s",
                usb_port.c_str());
    RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "baud_rate: %d",
                baud_rate);

    if (!dynamixel_workbench_.init(usb_port.c_str(), baud_rate, &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
      return CallbackReturn::ERROR;
    }

    for (uint i = 0; i < info_.joints.size(); ++i)
    {
      uint16_t model_number = 0;
      if (!dynamixel_workbench_.ping(joint_ids_[i], &model_number, &log))
      {
        RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
        return CallbackReturn::ERROR;
      }
    }

    enable_torque(false);
    set_control_mode(ControlMode::Position, true);
    enable_torque(true);

    const ControlItem *goal_position =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalPositionItem);
    if (goal_position == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    const ControlItem *goal_velocity =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kGoalVelocityItem);
    if (goal_velocity == nullptr)
    {
      goal_velocity =
          dynamixel_workbench_.getItemInfo(joint_ids_[0], kMovingSpeedItem);
    }
    if (goal_velocity == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    const ControlItem *present_position =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentPositionItem);
    if (present_position == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    const ControlItem *present_velocity =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentVelocityItem);
    if (present_velocity == nullptr)
    {
      present_velocity =
          dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentSpeedItem);
    }
    if (present_velocity == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    const ControlItem *present_current =
        dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentCurrentItem);
    if (present_current == nullptr)
    {
      present_current =
          dynamixel_workbench_.getItemInfo(joint_ids_[0], kPresentLoadItem);
    }
    if (present_current == nullptr)
    {
      return CallbackReturn::ERROR;
    }

    control_items_[kGoalPositionItem] = goal_position;
    control_items_[kGoalVelocityItem] = goal_velocity;
    control_items_[kPresentPositionItem] = present_position;
    control_items_[kPresentVelocityItem] = present_velocity;
    control_items_[kPresentCurrentItem] = present_current;

    if (!dynamixel_workbench_.addSyncWriteHandler(
            control_items_[kGoalPositionItem]->address,
            control_items_[kGoalPositionItem]->data_length, &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
      return CallbackReturn::ERROR;
    }

    if (!dynamixel_workbench_.addSyncWriteHandler(
            control_items_[kGoalVelocityItem]->address,
            control_items_[kGoalVelocityItem]->data_length, &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
      return CallbackReturn::ERROR;
    }

    uint16_t start_address =
        std::min(control_items_[kPresentPositionItem]->address,
                 control_items_[kPresentCurrentItem]->address);
    uint16_t read_length = control_items_[kPresentPositionItem]->data_length +
                           control_items_[kPresentVelocityItem]->data_length +
                           control_items_[kPresentCurrentItem]->data_length + 2;
    if (!dynamixel_workbench_.addSyncReadHandler(start_address, read_length,
                                                 &log))
    {
      RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  Omron1SHardware::export_state_interfaces()
  {
    RCLCPP_DEBUG(rclcpp::get_logger(Omron1SHardware),
                 "export_state_interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &joints_[i].state.position));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
          &joints_[i].state.velocity));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
          &joints_[i].state.effort));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  Omron1SHardware::export_command_interfaces()
  {
    RCLCPP_DEBUG(rclcpp::get_logger(Omron1SHardware),
                 "export_command_interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &joints_[i].command.position));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
          &joints_[i].command.velocity));
    }

    return command_interfaces;
  }

  CallbackReturn
  Omron1SHardware::on_activate(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(Omron1SHardware), "start");
    for (uint i = 0; i < joints_.size(); i++)
    {
      if (use_dummy_ && std::isnan(joints_[i].state.position))
      {
        joints_[i].state.position = 0.0;
        joints_[i].state.velocity = 0.0;
        joints_[i].state.effort = 0.0;
      }
    }
    read();
    reset_command();
    write();

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn Omron1SHardware::on_deactivate(
      const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_DEBUG(rclcpp::get_logger(Omron1SHardware), "stop");
    return CallbackReturn::SUCCESS;
  }

  return_type Omron1SHardware::read()
  {
    if (use_dummy_)
    {
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "read");
      return return_type::OK;
    }

    std::vector<uint8_t> ids(info_.joints.size(), 0);
    std::vector<int32_t> positions(info_.joints.size(), 0);
    std::vector<int32_t> velocities(info_.joints.size(), 0);
    std::vector<int32_t> currents(info_.joints.size(), 0);

    std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
    const char *log = nullptr;

    if (!dynamixel_workbench_.syncRead(kPresentPositionVelocityCurrentIndex,
                                       ids.data(), ids.size(), &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omron1SHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentCurrentItem]->address,
            control_items_[kPresentCurrentItem]->data_length, currents.data(),
            &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omron1SHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentVelocityItem]->address,
            control_items_[kPresentVelocityItem]->data_length, velocities.data(),
            &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omron1SHardware), "%s", log);
    }

    if (!dynamixel_workbench_.getSyncReadData(
            kPresentPositionVelocityCurrentIndex, ids.data(), ids.size(),
            control_items_[kPresentPositionItem]->address,
            control_items_[kPresentPositionItem]->data_length, positions.data(),
            &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omron1SHardware), "%s", log);
    }

    for (uint i = 0; i < ids.size(); i++)
    {
      joints_[i].state.position =
          dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
      joints_[i].state.velocity =
          dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
      joints_[i].state.effort =
          dynamixel_workbench_.convertValue2Current(currents[i]);
    }

    return return_type::OK;
  }

  return_type Omron1SHardware::write()
  {
    if (use_dummy_)
    {
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "write");
      for (auto &joint : joints_)
      {
        joint.state.position = joint.command.position;
      }

      return return_type::OK;
    }

    std::vector<uint8_t> ids(info_.joints.size(), 0);
    std::vector<int32_t> commands(info_.joints.size(), 0);

    std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());
    const char *log = nullptr;

    if (std::any_of(joints_.cbegin(), joints_.cend(),
                    [](auto j)
                    { return j.command.velocity != 0.0; }))
    {
      // Velocity control
      set_control_mode(ControlMode::Velocity);
      for (uint i = 0; i < ids.size(); i++)
      {
        commands[i] = dynamixel_workbench_.convertVelocity2Value(
            ids[i], static_cast<float>(joints_[i].command.velocity));
      }
      if (!dynamixel_workbench_.syncWrite(kGoalVelocityIndex, ids.data(),
                                          ids.size(), commands.data(), 1, &log))
      {
        RCLCPP_ERROR(rclcpp::get_logger(Omron1SHardware), "%s", log);
      }
      return return_type::OK;
    }
    else if (std::any_of(joints_.cbegin(), joints_.cend(),
                         [](auto j)
                         { return j.command.effort != 0.0; }))
    {
      // Effort control
      RCLCPP_ERROR(rclcpp::get_logger(Omron1SHardware),
                   "Effort control is not implemented");
      return return_type::ERROR;
    }

    // Position control
    set_control_mode(ControlMode::Position);
    for (uint i = 0; i < ids.size(); i++)
    {
      commands[i] = dynamixel_workbench_.convertRadian2Value(
          ids[i], static_cast<float>(joints_[i].command.position));
    }
    if (!dynamixel_workbench_.syncWrite(kGoalPositionIndex, ids.data(),
                                        ids.size(), commands.data(), 1, &log))
    {
      RCLCPP_ERROR(rclcpp::get_logger(Omron1SHardware), "%s", log);
    }

    return return_type::OK;
  }

  return_type Omron1SHardware::enable_torque(const bool enabled)
  {
    const char *log = nullptr;

    if (enabled && !torque_enabled_)
    {
      for (uint i = 0; i < info_.joints.size(); ++i)
      {
        if (!dynamixel_workbench_.torqueOn(joint_ids_[i], &log))
        {
          RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      reset_command();
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "Torque enabled");
    }
    else if (!enabled && torque_enabled_)
    {
      for (uint i = 0; i < info_.joints.size(); ++i)
      {
        if (!dynamixel_workbench_.torqueOff(joint_ids_[i], &log))
        {
          RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "Torque disabled");
    }

    torque_enabled_ = enabled;
    return return_type::OK;
  }

  return_type Omron1SHardware::set_control_mode(const ControlMode &mode,
                                                const bool force_set)
  {
    const char *log = nullptr;

    if (mode == ControlMode::Velocity &&
        (force_set || control_mode_ != ControlMode::Velocity))
    {
      bool torque_enabled = torque_enabled_;
      if (torque_enabled)
      {
        enable_torque(false);
      }

      for (uint i = 0; i < joint_ids_.size(); ++i)
      {
        if (!dynamixel_workbench_.setVelocityControlMode(joint_ids_[i], &log))
        {
          RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "Velocity control");
      control_mode_ = ControlMode::Velocity;

      if (torque_enabled)
      {
        enable_torque(true);
      }
    }
    else if (mode == ControlMode::Position &&
             (force_set || control_mode_ != ControlMode::Position))
    {
      bool torque_enabled = torque_enabled_;
      if (torque_enabled)
      {
        enable_torque(false);
      }

      for (uint i = 0; i < joint_ids_.size(); ++i)
      {
        if (!dynamixel_workbench_.setPositionControlMode(joint_ids_[i], &log))
        {
          RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware), "%s", log);
          return return_type::ERROR;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger(Omron1SHardware), "Position control");
      control_mode_ = ControlMode::Position;

      if (torque_enabled)
      {
        enable_torque(true);
      }
    }
    else if (control_mode_ != ControlMode::Velocity &&
             control_mode_ != ControlMode::Position)
    {
      RCLCPP_FATAL(rclcpp::get_logger(Omron1SHardware),
                   "Only position/velocity control are implemented");
      return return_type::ERROR;
    }

    return return_type::OK;
  }

  return_type Omron1SHardware::reset_command()
  {
    for (uint i = 0; i < joints_.size(); i++)
    {
      joints_[i].command.position = joints_[i].state.position;
      joints_[i].command.velocity = 0.0;
      joints_[i].command.effort = 0.0;
    }

    return return_type::OK;
  }

  int Omron1SHardware::R88Dsetup(uint16 slave)
  {
    int retval = 0;
    // uint16 u16val;
    int8 i8val;

    /* Map velocity PDO assignment via Complete Access*/
    //  uint16 map_1c12[4] = {0x0003, 0x1601, 0x1602, 0x1604};
    //  uint16 map_1c13[3] = {0x0002, 0x1a01, 0x1a03};
    //  retval += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTRXM);
    //  retval += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTRXM);

    // /* set some motor parameters, just as example */
    // u16val = 1200; // max motor current in mA
    // retval += ec_SDOwrite(slave, 0x8010, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    // u16val = 150; // motor coil resistance in 0.01ohm
    // retval += ec_SDOwrite(slave, 0x8010, 0x04, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    // /* set other necessary parameters as needed */

    // set to cyclic synchronous position mode
    i8val = 8;
    retval += ec_SDOwrite(slave, 0x3000, 0xf2, FALSE, sizeof(i8val), &i8val, EC_TIMEOUTRXM);

    printf("R88D slave %d set, retval = %d\n", slave, retval);
    return 1;
  }

  OSAL_THREAD_FUNC Omron1SHardware::ecatcheck(void *ptr)
  {
    int slave;
    (void)ptr; /* Not used */
    currentgroup = 0;
    while (1)
    {
      if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
      {
        if (needlf)
        {
          needlf = FALSE;
          printf("\n");
        }
        /* one ore more slaves are not responding */
        ec_group[currentgroup].docheckstate = FALSE;
        ec_readstate();
        for (slave = 1; slave <= ec_slavecount; slave++)
        {
          if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
          {
            ec_group[currentgroup].docheckstate = TRUE;
            if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
            {
              printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
              ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
              ec_writestate(slave);
            }
            else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
            {
              printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
              ec_slave[slave].state = EC_STATE_OPERATIONAL;
              ec_writestate(slave);
            }
            else if (ec_slave[slave].state > EC_STATE_NONE)
            {
              if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
              {
                ec_slave[slave].islost = FALSE;
                printf("MESSAGE : slave %d reconfigured\n", slave);
              }
            }
            else if (!ec_slave[slave].islost)
            {
              /* re-check state */
              ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
              if (ec_slave[slave].state == EC_STATE_NONE)
              {
                ec_slave[slave].islost = TRUE;
                printf("ERROR : slave %d lost\n", slave);
              }
            }
          }
          if (ec_slave[slave].islost)
          {
            if (ec_slave[slave].state == EC_STATE_NONE)
            {
              if (ec_recover_slave(slave, EC_TIMEOUTMON))
              {
                ec_slave[slave].islost = FALSE;
                printf("MESSAGE : slave %d recovered\n", slave);
              }
            }
            else
            {
              ec_slave[slave].islost = FALSE;
              printf("MESSAGE : slave %d found\n", slave);
            }
          }
        }
        if (!ec_group[currentgroup].docheckstate)
          printf("OK : all slaves resumed OPERATIONAL.\n");
      }
      osal_usleep(10000);
    }
  }

} // namespace omron1s_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omron1s_hardware::Omron1SHardware,
                       hardware_interface::SystemInterface)