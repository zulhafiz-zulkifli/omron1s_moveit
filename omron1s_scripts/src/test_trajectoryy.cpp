#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <tf2/LinearMath/Quaternion.h>

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
    move_group_arm.startStateMonitor();
    // get_info();

    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TestTrajectory::timer_callback, this), timer_cb_group_);

  } // end of constructor

  // Getting Basic Information
  void get_info() {

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_arm.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "Pose reference frame: %s", move_group_arm.getPoseReferenceFrame().c_str());
    RCLCPP_INFO(LOGGER, "End-effector link: %s", move_group_arm.getEndEffectorLink().c_str());
 
    
    
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_arm.getJointModelGroupNames().begin(), move_group_arm.getJointModelGroupNames().end(),
                std::ostream_iterator<std::string>(std::cout, ", "));
  
  }

  void current_state() {
    // move_group_arm.setStartStateToCurrentState();
    // RCLCPP_INFO(LOGGER, "Get Robot Current State");

    // current_state_arm = move_group_arm.getCurrentState(10);

    // current_state_arm->copyJointGroupPositions(this->joint_model_group_arm,
    //                                            this->joint_group_positions_arm);

    // RCLCPP_INFO(LOGGER, "%f", joint_group_positions_arm[0]);
    // RCLCPP_INFO(LOGGER, "%f", joint_group_positions_arm[1]);
    // RCLCPP_INFO(LOGGER, "%f", joint_group_positions_arm[2]);
    // RCLCPP_INFO(LOGGER, "%f", joint_group_positions_arm[3]);

    RCLCPP_INFO(LOGGER, "Current Pose:");
    auto posee = move_group_arm.getCurrentPose().pose;
    // for (double i: rpy){
    //   RCLCPP_INFO(LOGGER, "%f", i);
    // }
    RCLCPP_INFO(LOGGER, "%f", posee.position.x);
    RCLCPP_INFO(LOGGER, "%f", posee.position.y);
    RCLCPP_INFO(LOGGER, "%f", posee.position.z);

  }


  void plan_arm_joint_space1() {

    // RCLCPP_INFO(LOGGER, "Planning to Joint Space");
    // joint_group_positions_arm[0] = -2.50;
    // joint_group_positions_arm[1] = 1.50;
    // joint_group_positions_arm[2] = -1.50;
    // joint_group_positions_arm[3] = -1.55;
    // move_group_arm.setJointValueTarget(joint_group_positions_arm);

    // tf2::Quaternion q;
    // // Create a quaternion from roll/pitch/yaw in radians (0, 0, 0)
    // // [ x: 1.5707963, y: 0.2381838, z: -3.1415927 ]
    // q.setRPY(-1*1.57, 0.00, -3.14);
    // // Print the quaternion components (0, 0, 0, 1)
    // // RCLCPP_INFO(this->get_logger(), "%f %f %f %f", q.x(), q.y(), q.z(), q.w());

    // RCLCPP_INFO(LOGGER, "Planning to End-Effector Pose");
    // target_pose.orientation.x = q.x();
    // target_pose.orientation.y = q.y();
    // target_pose.orientation.z = q.z();
    // target_pose.orientation.w = q.w();
    // target_pose.position.x = 0.0;
    // target_pose.position.y = -0.5;
    // target_pose.position.z = 0.5;

    // move_group_arm.setApproximateJointValueTarget(target_pose,"link4");

    // auto current_pose = move_group_arm.getCurrentPose();
    // moveit_msgs::msg::OrientationConstraint orientation_constraint;
    // orientation_constraint.header.frame_id = move_group_arm.getPoseReferenceFrame();
    // orientation_constraint.link_name = move_group_arm.getEndEffectorLink();

    // // Quaternion desired: -0.701115 -0.082895 0.082895 0.702886, 
    // // quaternion actual: -0.083767 0.702151 -0.702106 0.083752, 
    // // error: x=3.139817, y=0.002384, z=3.141434, 

    // orientation_constraint.orientation.x = current_pose.pose.orientation.y;
    // orientation_constraint.orientation.y = current_pose.pose.orientation.w;
    // orientation_constraint.orientation.z = current_pose.pose.orientation.x;
    // orientation_constraint.orientation.w = current_pose.pose.orientation.z;
    // // orientation_constraint.orientation.x = -0.702000;
    // // orientation_constraint.orientation.y = -0.083000;
    // // orientation_constraint.orientation.z = 0.083000;
    // // orientation_constraint.orientation.w = 0.702000;
    


    // orientation_constraint.absolute_x_axis_tolerance = 0.2;
    // orientation_constraint.absolute_y_axis_tolerance = 3.141593;
    // orientation_constraint.absolute_z_axis_tolerance = 3.141593;
    // orientation_constraint.weight = 1.0;
    
    // moveit_msgs::msg::Constraints orientation_constraints;
    // orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);

    // move_group_arm.setPathConstraints(orientation_constraints);

    move_group_arm.setPositionTarget(0.000000, -0.500000, 0.600000, "link4");



    bool plan_success = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    if(plan_success){
      move_group_arm.execute(my_plan_arm);
    }

    move_group_arm.clearPoseTargets();
    

  }

    void plan_arm_joint_space2() {

    RCLCPP_INFO(LOGGER, "Planning to Joint Space");
    // joint_group_positions_arm[4] = joint_group_positions_arm[4];
    // move_group_arm.setJointValueTarget(joint_group_positions_arm);

    std::vector< double> cur_joint_values = move_group_arm.getCurrentJointValues();
    // for (double i: rpy){
    //   RCLCPP_INFO(LOGGER, "%f", i);
    // }
    // cur_joint_values[3] = cur_joint_values[3]+0.785398;
    move_group_arm.setJointValueTarget(cur_joint_values);

    // RCLCPP_INFO(LOGGER, "Planning to End-Effector Pose");
    // // target_pose.orientation.x = 0.084;
    // // target_pose.orientation.y = -0.702;
    // // target_pose.orientation.z = 0.702;
    // // target_pose.orientation.w = -0.084;
    // // target_pose.position.x = 0.0;
    // // target_pose.position.y = 0.5;
    // // target_pose.position.z = 0.5;
    // // // move_group_arm.setPoseTarget(target_pose);
    // // std::string i = "link4";
    // // move_group_arm.setApproximateJointValueTarget(target_pose,i);

    // // - Translation: [0.201, -0.418, 0.448]
    // // - Rotation: in Quaternion [0.084, -0.702, 0.702, -0.084]

    // move_group_arm.setPositionTarget(0.0, 0.5, 0.5, "link4");



    bool plan_success = (move_group_arm.plan(my_plan_arm) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Execute
    if(plan_success){
      move_group_arm.execute(my_plan_arm);
    }
    

  }

  // Timer Callback function
  void timer_callback() {

    // this->timer_->cancel();
    
    current_state();
    // plan_arm_joint_space1();
    // current_state();
    // plan_arm_joint_space2();


  }

private:
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  std::vector<double> joint_group_positions_arm;
  geometry_msgs::msg::Pose target_pose;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  rclcpp::TimerBase::SharedPtr timer_;

  moveit::planning_interface::MoveGroupInterface move_group_arm;

  const moveit::core::JointModelGroup *joint_model_group_arm;

  moveit::core::RobotStatePtr current_state_arm;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;

}; // End of Class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_demo", node_options);
  std::shared_ptr<TestTrajectory> planner_node = std::make_shared<TestTrajectory>(move_group_node);

  rclcpp::executors::MultiThreadedExecutor planner_executor;
  planner_executor.add_node(planner_node);
  planner_executor.spin();

  // rclcpp::executors::SingleThreadedExecutor planner_executor;
  // planner_executor.add_node(planner_node);
  // std::thread([&planner_executor]() { planner_executor.spin(); }).detach();

  rclcpp::shutdown();
  return 0;


  
}