#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <tf2/LinearMath/Quaternion.h>

static const double PLANNING_TIME_S = 5.0;
static const double MAX_VELOCITY_SCALE = 1.0;
static const double MAX_ACCELERATION_SCALE = 1.0;
static const unsigned int PLANNING_ATTEMPTS = 20;
static const double GOAL_TOLERANCE = 1e-3;
static const std::string PLANNING_GROUP_ARM = "omron1s_manipulator";


class MoveRobot : public rclcpp::Node {
public:
    explicit MoveRobot(const rclcpp::NodeOptions &options);

private:
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    void doTask1();
    void doTask2();

    // void service_example_callback(const std::shared_ptr<your_service_msgs::srv::ExampleService::Request> request,
    //                               std::shared_ptr<your_service_msgs::srv::ExampleService::Response> response);
};

MoveRobot::MoveRobot(const rclcpp::NodeOptions &options)
        : Node("example_services_node", options), node_(std::make_shared<rclcpp::Node>("example_group_node")),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {

    
    // service_example_server_ = this->create_service<your_service_msgs::srv::ExampleService>(
    //         "~/example_service",
    //         std::bind(&MoveRobot::service_example_callback, this, std::placeholders::_1, std::placeholders::_2)
    // );


    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP_ARM);
    move_group_->setPoseReferenceFrame("world");
    move_group_->setEndEffectorLink("tool0");
    // move_group_->setWorkspace(-5.0, -5.0, -5.0, 5.0, 5.0, 5.0);
    // move_group_->setGoalOrientationTolerance(99999999);
    // move_group_->setGoalJointTolerance(10000);
    move_group_->setGoalTolerance(999999999.0);
    // move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
    // move_group_->setPlanningTime(20.0);
    move_group_->startStateMonitor();

    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "Pose reference frame: %s", move_group_->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End-effector link: %s", move_group_->getEndEffectorLink().c_str());

    // move_group_->setPlanningTime(PLANNING_TIME_S);
   
    
    // move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    // move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MoveRobot::timer_callback, this));

    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
}

  void MoveRobot::timer_callback() {
    timer_->cancel();
    doTask1();
    // doTask2();

    // geometry_msgs::msg::Pose posee = move_group_->getCurrentPose().pose;
    // RCLCPP_INFO(this->get_logger(), "Current Pose:");
    // RCLCPP_INFO(this->get_logger(), "%f", posee.position.x);
    // RCLCPP_INFO(this->get_logger(), "%f", posee.position.y);
    // RCLCPP_INFO(this->get_logger(), "%f", posee.position.z);

  }

    void MoveRobot::doTask1() {
      RCLCPP_INFO(this->get_logger(), "Perform Task1");

      // RCLCPP_INFO(LOGGER, "Planning to Joint Space");
      // joint_group_positions_arm[0] = -2.50;
      // joint_group_positions_arm[1] = 1.50;
      // joint_group_positions_arm[2] = -1.50;
      // joint_group_positions_arm[3] = -1.55;
      // move_group_->setJointValueTarget(joint_group_positions_arm);

      // tf2::Quaternion q;
      // // Create a quaternion from roll/pitch/yaw in radians (0, 0, 0)
      // // [ x: 1.5707963, y: 0.2381838, z: -3.1415927 ]
      // q.setRPY(-1*1.57, 0.00, -3.14);
      // // Print the quaternion components (0, 0, 0, 1)
      // // RCLCPP_INFO(this->get_logger(), "%f %f %f %f", q.x(), q.y(), q.z(), q.w());


      // auto current_pose = move_group_->getCurrentPose();
      // moveit_msgs::msg::OrientationConstraint orientation_constraint;
      // orientation_constraint.header.frame_id = move_group_->getPoseReferenceFrame();
      // orientation_constraint.link_name = move_group_->getEndEffectorLink();
      // orientation_constraint.orientation.x = current_pose.pose.orientation.x;
      // orientation_constraint.orientation.y = current_pose.pose.orientation.y;
      // orientation_constraint.orientation.z = current_pose.pose.orientation.z;
      // orientation_constraint.orientation.w = current_pose.pose.orientation.w;
      // orientation_constraint.absolute_x_axis_tolerance = 100;
      // orientation_constraint.absolute_y_axis_tolerance = 100;
      // orientation_constraint.absolute_z_axis_tolerance = 100;
      // orientation_constraint.weight = 1.0;
      // moveit_msgs::msg::Constraints orientation_constraints;
      // orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
      // move_group_->setPathConstraints(orientation_constraints);

      // geometry_msgs::msg::Pose target_pose;
      // // target_pose.orientation.x = current_pose.pose.orientation.x;
      // // target_pose.orientation.y = current_pose.pose.orientation.y;
      // // target_pose.orientation.z = current_pose.pose.orientation.z;
      // // target_pose.orientation.w = current_pose.pose.orientation.w;
      // target_pose.position.x = 0.25;
      // target_pose.position.y = 0.25;
      // target_pose.position.z = 0.25;
      // move_group_->setApproximateJointValueTarget(target_pose,"tool0");

      move_group_->setPositionTarget(0.500000, 0.500000, 0.500000, "tool0");

      bool plan_success = (move_group_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // Execute
      if(plan_success){
        move_group_->execute(my_plan_arm);
      }

      // move_group_->clearPoseTargets();
    

  }

    void MoveRobot::doTask2() {
      RCLCPP_INFO(this->get_logger(), "Perform Task2");

      // RCLCPP_INFO(LOGGER, "Planning to Joint Space");
      // joint_group_positions_arm[4] = joint_group_positions_arm[4];
      // move_group_->setJointValueTarget(joint_group_positions_arm);

      

      // RCLCPP_INFO(LOGGER, "Planning to End-Effector Pose");
      // // target_pose.orientation.x = 0.084;
      // // target_pose.orientation.y = -0.702;
      // // target_pose.orientation.z = 0.702;
      // // target_pose.orientation.w = -0.084;
      // // target_pose.position.x = 0.0;
      // // target_pose.position.y = 0.5;
      // // target_pose.position.z = 0.5;
      // // // move_group_->setPoseTarget(target_pose);
      // // std::string i = "link4";
      // // move_group_->setApproximateJointValueTarget(target_pose,i);

      // // - Translation: [0.201, -0.418, 0.448]
      // // - Rotation: in Quaternion [0.084, -0.702, 0.702, -0.084]

      // move_group_->setPositionTarget(0.0, 0.5, 0.5, "link4");

      std::vector< double> cur_joint_values = move_group_->getCurrentJointValues();
      cur_joint_values[3] = cur_joint_values[3]+0.785398;
      move_group_->setJointValueTarget(cur_joint_values);

      bool plan_success = (move_group_->plan(my_plan_arm) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // Execute
      if(plan_success){
        move_group_->execute(my_plan_arm);
      }
      

  }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<MoveRobot>(node_options);
    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
    return 0;
}