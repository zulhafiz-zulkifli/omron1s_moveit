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
static const unsigned int PLANNING_ATTEMPTS = 5;
static const double GOAL_TOLERANCE = 1e-3;
static const std::string PLANNING_GROUP_ARM = "omron1s_manipulator";


class MoveRobot : public rclcpp::Node {
public:
    explicit MoveRobot(const rclcpp::NodeOptions &options);

private:
    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Service<your_service_msgs::srv::ExampleService>::SharedPtr service_example_server_;
    // std::string node_namespace_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
    void timer_callback();

    // void service_example_callback(const std::shared_ptr<your_service_msgs::srv::ExampleService::Request> request,
    //                               std::shared_ptr<your_service_msgs::srv::ExampleService::Response> response);
};

MoveRobot::MoveRobot(const rclcpp::NodeOptions &options)
        : Node("example_services_node", options), node_(std::make_shared<rclcpp::Node>("example_group_node")),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
    // node_namespace_ = ((std::string) this->get_namespace()).erase(0, 1);
    
    // service_example_server_ = this->create_service<your_service_msgs::srv::ExampleService>(
    //         "~/example_service",
    //         std::bind(&MoveRobot::service_example_callback, this, std::placeholders::_1, std::placeholders::_2)
    // );

    // auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
    //         node_namespace_ + "_ur_manipulator",
    //         "/" + node_namespace_,
    //         "robot_description");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP_ARM);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&MoveRobot::timer_callback, this));
    //move_group_->setPoseReferenceFrame("base_link_inertia");
    // move_group_->setPlanningTime(PLANNING_TIME_S);
    // move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
    // move_group_->setGoalTolerance(GOAL_TOLERANCE);
    // move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    // move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
}

  void MoveRobot::timer_callback() {

    geometry_msgs::msg::Pose posee = move_group_->getCurrentPose().pose;

    RCLCPP_INFO(this->get_logger(), "Current Pose:");
    RCLCPP_INFO(this->get_logger(), "%f", posee.position.x);
    RCLCPP_INFO(this->get_logger(), "%f", posee.position.y);
    RCLCPP_INFO(this->get_logger(), "%f", posee.position.z);

  }

// void MoveRobot::service_example_callback(const std::shared_ptr<your_service_msgs::srv::ExampleService::Request> request,
//                                          std::shared_ptr<your_service_msgs::srv::ExampleService::Response> response) {
//     move_group_->setStartStateToCurrentState();
//     geometry_msgs::msg::Pose pose_goal = move_group_->getCurrentPose().pose;
//     pose_goal.position.x += request->transform.translation.x;
//     pose_goal.position.y += request->transform.translation.y;
//     pose_goal.position.z += request->transform.translation.z;
//     tf2::Quaternion q_orig(pose_goal.orientation.x, pose_goal.orientation.y, pose_goal.orientation.z,
//                            pose_goal.orientation.w);
//     tf2::Quaternion q_rot(request->transform.rotation.x, request->transform.rotation.y, request->transform.rotation.z,
//                           request->transform.rotation.w);
//     tf2::Quaternion q_new;
//     q_new = q_rot * q_orig;
//     q_new.normalize();
//     pose_goal.orientation.x = q_new.x();
//     pose_goal.orientation.y = q_new.y();
//     pose_goal.orientation.z = q_new.z();
//     pose_goal.orientation.w = q_new.w();
//     move_group_->setPoseTarget(pose_goal);
//     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
//     RCLCPP_INFO(this->get_logger(), "Planned position x: %f, y: %f, z: %f", pose_goal.position.x, pose_goal.position.y,
//                 pose_goal.position.z);
//     bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//     response->success = success;
//     move_group_->move();
// }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<MoveRobot>(node_options);
    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
    return 0;
}