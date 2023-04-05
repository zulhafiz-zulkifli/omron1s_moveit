#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "rclcpp/rclcpp.hpp"
#include <string>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_ARM = "omron1s_manipulator";

class TestTrajectory : public rclcpp::Node {
public:
  TestTrajectory(std::shared_ptr<rclcpp::Node> move_group_node)
      : Node("test_trajectory"),
        move_group_arm(move_group_node, PLANNING_GROUP_ARM),
        joint_model_group_arm(
            move_group_arm.getCurrentState()->getJointModelGroup(
                PLANNING_GROUP_ARM)) {

    this->timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&TestTrajectory::timer_callback, this));

  } // end of constructor

  // Getting Basic Information
  void get_info() {

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group_arm.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_arm.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
  
  }

  void current_state() {
    RCLCPP_INFO(LOGGER, "Get Robot Current State");

    current_state_arm = move_group_arm.getCurrentState(10);

    current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
                                               this->joint_group_positions_arm);
  }

  void go_position1() {

    RCLCPP_INFO(LOGGER, "Planning to Joint Space");

    joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
    joint_group_positions_arm[1] = 1.145;  // Shoulder Lift
    joint_group_positions_arm[2] = -1.145;  // Elbow
    joint_group_positions_arm[3] = 0.00;  // Wrist 1
    //joint_group_positions_arm[4] = 0.00;  // Wrist 2
    //#joint_group_positions_arm[5] = 0.00;  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    move_group_arm.execute(my_plan_arm);
  }

  void go_position2() {

    RCLCPP_INFO(LOGGER, "Planning to Joint Space");

    joint_group_positions_arm[0] = 3.14;  // Shoulder Pan
    joint_group_positions_arm[1] = 1.145;  // Shoulder Lift
    joint_group_positions_arm[2] = -1.145;  // Elbow
    joint_group_positions_arm[3] = 0.00;  // Wrist 1
    //joint_group_positions_arm[4] = 0.00;  // Wrist 2
    //joint_group_positions_arm[5] = 1.57;  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    bool success_arm = (move_group_arm.move() ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

  }

    void go_position3() {

    RCLCPP_INFO(LOGGER, "Planning to Joint Space");

    joint_group_positions_arm[0] = 0.00;  // Shoulder Pan
    joint_group_positions_arm[1] = 0.00;  // Shoulder Lift
    joint_group_positions_arm[2] = 0.00;  // Elbow
    joint_group_positions_arm[3] = 0.00;  // Wrist 1
    //joint_group_positions_arm[4] = 0.00;  // Wrist 2
    //joint_group_positions_arm[5] = 1.57;  // Wrist 3

    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    bool success_arm = (move_group_arm.move() ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

  }

  // Timer Callback function
  void timer_callback() {

    this->timer_->cancel();
    get_info();
    current_state();
    go_position1();
    current_state();
    go_position2();
    current_state();
    go_position3();
  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_group_positions_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  rclcpp::TimerBase::SharedPtr timer_;

  moveit::planning_interface::MoveGroupInterface move_group_arm;

  const moveit::core::JointModelGroup *joint_model_group_arm;

  moveit::core::RobotStatePtr current_state_arm;

}; // End of Class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group_demo", node_options);

  rclcpp::executors::SingleThreadedExecutor planner_executor;
  std::shared_ptr<TestTrajectory> planner_node =
      std::make_shared<TestTrajectory>(move_group_node);
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  rclcpp::shutdown();
  return 0;
}