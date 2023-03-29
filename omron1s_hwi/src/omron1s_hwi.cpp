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

  int R88Dsetup(uint16 slave)
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

  return_type Omron1SHardware::configure(const hardware_interface::HardwareInfo &info)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("Omron1SHardware"), "configure");
    if (configure_default(info) != return_type::OK)
    {
      return return_type::ERROR;
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
      RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "joint_id %d: %d", i,
                  joint_ids_[i]);
    }

    if (info_.hardware_parameters.find("use_dummy") !=
            info_.hardware_parameters.end() &&
        info_.hardware_parameters.at("use_dummy") == "true")
    {
      use_dummy_ = true;
      RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "dummy mode");
      status_ = hardware_interface::status::CONFIGURED;
      return return_type::OK;
    }

    RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "SOEM (Simple Open EtherCAT Master)");
    /* create thread to handle slave error handling in OP */
    // osal_thread_create(&thread1, 128000, &ecatcheck, (void *)&ctime);

    needlf = FALSE;
    inOP = FALSE;

    /////////////////////////////////////////////////////////////////////////////////////////
    /* initialise SOEM, bind socket to ifname */
    auto interface_name = info_.hardware_parameters.at("interface_name");
    RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "interface_name: %s", interface_name.c_str());
    if (ec_init(interface_name.c_str()))
    {
      RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "ec_init on %s succeeded.", interface_name.c_str());
      /* find and auto-config slaves */

      if (ec_config_init(FALSE) > 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "%d slaves found and configured.\n", ec_slavecount);
        if (ec_slavecount == info_.joints.size())
        {
          for (int i = 1; i <= ec_slavecount; i++)
          {
            if (strcmp(ec_slave[i].name, "R88D-1SN01H-ECT") || strcmp(ec_slave[i].name, "R88D-1SN02H-ECT"))
            {
              RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "Found %s at position %d", ec_slave[i].name, i);
              /* link slave specific setup to preop->safeop hook */
              ec_slave[i].PO2SOconfig = R88Dsetup;
            }
          }
          ec_config_map(&IOmap);
          ec_configdc();
          RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "Slaves mapped, state to SAFE_OP.");
          /* wait for all slaves to reach SAFE_OP state */
          ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
          RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "Request operational state for all slaves.");
          expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
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
            RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "Operational state reached for all slaves.");
            status_ = hardware_interface::status::CONFIGURED;
            return return_type::OK;
          }
          else
          {
            RCLCPP_FATAL(rclcpp::get_logger("Omron1SHardware"), "Not all slaves reached operational state.");
            ec_close();
            return return_type::ERROR;

            // printf("Not all slaves reached operational state.\n");
            // ec_readstate();
            // for (int i = 1; i <= ec_slavecount; i++)
            // {
            //   if (ec_slave[i].state != EC_STATE_OPERATIONAL)
            //   {
            //     printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
            //            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            //   }
            // }
          }
          // printf("\nRequest init state for all slaves\n");
          // ec_slave[0].state = EC_STATE_INIT;
          // /* request INIT state for all slaves */
          // ec_writestate(0);
        }
        else
        {
          RCLCPP_FATAL(rclcpp::get_logger("Omron1SHardware"), "More or less than %lu slaves found!", info_.joints.size());
          ec_close();
          return return_type::ERROR;
        }
      }
      else
      {
        RCLCPP_FATAL(rclcpp::get_logger("Omron1SHardware"), "No slaves found!");
        ec_close();
        return return_type::ERROR;
      }
    }
    else
    {
      RCLCPP_FATAL(rclcpp::get_logger("Omron1SHardware"), "No socket connection on %s", interface_name.c_str());
      return return_type::ERROR;
    }

    /////////////////////////////////////////////////////////////////////////////////////////
  }

  std::vector<hardware_interface::StateInterface> Omron1SHardware::export_state_interfaces()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("Omron1SHardware"),
                 "export_state_interfaces");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &joints_[i].state.position));
      // state_interfaces.emplace_back(hardware_interface::StateInterface(
      //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      //     &joints_[i].state.velocity));
      // state_interfaces.emplace_back(hardware_interface::StateInterface(
      //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      //     &joints_[i].state.effort));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> Omron1SHardware::export_command_interfaces()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("Omron1SHardware"),
                 "export_command_interfaces");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION,
          &joints_[i].command.position));
      // command_interfaces.emplace_back(hardware_interface::CommandInterface(
      //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      //     &joints_[i].command.velocity));
    }

    return command_interfaces;
  }

  return_type Omron1SHardware::start()
  {
    RCLCPP_DEBUG(rclcpp::get_logger("Omron1SHardware"), "activate");
    for (uint i = 0; i < joints_.size(); i++)
    {
      if (use_dummy_ && std::isnan(joints_[i].state.position))
      {
        joints_[i].state.position = 0.0;
        joints_[i].state.velocity = 0.0;
        joints_[i].state.effort = 0.0;
      }
    }
    inOP = TRUE;
    torque_enabled_ = true;
    read();
    reset_command();
    write();
    status_ = hardware_interface::status::STARTED;
    return return_type::OK;
  }

  return_type Omron1SHardware::stop()
  {
    inOP = FALSE;
    torque_enabled_ = false;
    RCLCPP_DEBUG(rclcpp::get_logger("Omron1SHardware"), "deactivate");
    status_ = hardware_interface::status::STOPPED;
    return return_type::OK;
  }

  return_type Omron1SHardware::read()
  {
    if (use_dummy_)
    {
      // RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "read");
      return return_type::OK;
    }

    for (int s = 1; s <= ec_slavecount; s++)
    {
      input_R88D = (input_R88Dt *)ec_slave[s].inputs;
      joints_[s - 1].state.position = input_R88D->position_actual_value * (2 * PI) / ENCODER_RESOLUTION;
      // joints_[i].state.velocity = 0.0;
      // joints_[i].state.effort = 0.0;
      // printf("slave %d pos : %10d     ", s, input_R88D->position_actual_value);
    }

    return return_type::OK;
  }

  return_type Omron1SHardware::write()
  {
    if (use_dummy_)
    {
      // RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"), "write");
      for (auto &joint : joints_)
      {
        joint.state.position = joint.command.position;
      }
      return return_type::OK;
    }

    for (int s = 1; s <= ec_slavecount; s++)
    {
      output_R88D = (output_R88Dt *)ec_slave[s].outputs;

      // if (torque_enabled_)
      // {
      //   if (output_R88D->controlword == 0)
      //   {
      //     output_R88D->controlword = 6;
      //   }
      //   else if (output_R88D->controlword == 6)
      //   {
      //     output_R88D->controlword = 7;
      //   }
      //   else if (output_R88D->controlword == 7)
      //   {
      //     output_R88D->controlword = 15;
      //   }
      // }
      // else
      // {
      //   if (output_R88D->controlword == 15)
      //   {
      //     output_R88D->controlword = 7;
      //   }
      //   else if (output_R88D->controlword == 7)
      //   {
      //     output_R88D->controlword = 6;
      //   }
      //   else if (output_R88D->controlword == 6)
      //   {
      //     output_R88D->controlword = 0;
      //   }
      // }
      if (output_R88D->controlword == 31)
      {
        output_R88D->controlword = 0;
        RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"),"Slave %d controlword set to 0", s);
      }
      else if (output_R88D->controlword == 0)
      {
        output_R88D->controlword = 6;
        RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"),"Slave %d controlword set to 6", s);
      }
      else if (output_R88D->controlword == 6)
      {
        output_R88D->controlword = 7;
        RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"),"Slave %d controlword set to 7", s);
      }
      else if (output_R88D->controlword == 7)
      {
        output_R88D->controlword = 15;
        RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"),"Slave %d torque on", s);
      }
      else if (output_R88D->controlword != 15)
      {
        output_R88D->controlword = 31;
        // RCLCPP_INFO(rclcpp::get_logger("Omron1SHardware"),"Slave %d controlword is %d", s, output_R88D->controlword);
      }

      output_R88D->target_position = int(joints_[s - 1].command.position * ENCODER_RESOLUTION / (2 * PI));
    }

    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    if (wkc >= expectedWKC)
    {
      needlf = TRUE;
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

  // OSAL_THREAD_FUNC Omron1SHardware::ecatcheck(void *ptr)
  // {
  //   int slave;
  //   (void)ptr; /* Not used */
  //   currentgroup = 0;
  //   while (1)
  //   {
  //     if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
  //     {
  //       if (needlf)
  //       {
  //         needlf = FALSE;
  //         printf("\n");
  //       }
  //       /* one ore more slaves are not responding */
  //       ec_group[currentgroup].docheckstate = FALSE;
  //       ec_readstate();
  //       for (slave = 1; slave <= ec_slavecount; slave++)
  //       {
  //         if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
  //         {
  //           ec_group[currentgroup].docheckstate = TRUE;
  //           if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
  //           {
  //             printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
  //             ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
  //             ec_writestate(slave);
  //           }
  //           else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
  //           {
  //             printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
  //             ec_slave[slave].state = EC_STATE_OPERATIONAL;
  //             ec_writestate(slave);
  //           }
  //           else if (ec_slave[slave].state > EC_STATE_NONE)
  //           {
  //             if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
  //             {
  //               ec_slave[slave].islost = FALSE;
  //               printf("MESSAGE : slave %d reconfigured\n", slave);
  //             }
  //           }
  //           else if (!ec_slave[slave].islost)
  //           {
  //             /* re-check state */
  //             ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
  //             if (ec_slave[slave].state == EC_STATE_NONE)
  //             {
  //               ec_slave[slave].islost = TRUE;
  //               printf("ERROR : slave %d lost\n", slave);
  //             }
  //           }
  //         }
  //         if (ec_slave[slave].islost)
  //         {
  //           if (ec_slave[slave].state == EC_STATE_NONE)
  //           {
  //             if (ec_recover_slave(slave, EC_TIMEOUTMON))
  //             {
  //               ec_slave[slave].islost = FALSE;
  //               printf("MESSAGE : slave %d recovered\n", slave);
  //             }
  //           }
  //           else
  //           {
  //             ec_slave[slave].islost = FALSE;
  //             printf("MESSAGE : slave %d found\n", slave);
  //           }
  //         }
  //       }
  //       if (!ec_group[currentgroup].docheckstate)
  //         printf("OK : all slaves resumed OPERATIONAL.\n");
  //     }
  //     osal_usleep(10000);
  //   }
  // }

} // namespace omron1s_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(omron1s_hardware::Omron1SHardware,
                       hardware_interface::SystemInterface)