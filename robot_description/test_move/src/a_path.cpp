#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "draw_letter_a",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("draw_letter_a");

  // Create the MoveGroupInterface
  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // Define waypoints to draw letter 'A'
  std::vector<geometry_msgs::msg::Pose> waypoints;
  
  geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;
  
  // Move to the start of 'A'
  geometry_msgs::msg::Pose pose1 = start_pose;
  pose1.position.x += 0.2;
  pose1.position.y -= 0.2;
  pose1.position.z += 0.3;
  waypoints.push_back(pose1);
  
  // Move diagonally up for the left leg of 'A'
  geometry_msgs::msg::Pose pose2 = pose1;
  pose2.position.x += 0.2;
  pose2.position.y += 0.4;
  pose2.position.z += 0.3;
  waypoints.push_back(pose2);
  
  // Move diagonally down for the right leg of 'A'
  geometry_msgs::msg::Pose pose3 = pose2;
  pose3.position.x += 0.2;
  pose3.position.y -= 0.4;
  waypoints.push_back(pose3);
  
  // Move horizontally for the middle bar of 'A'
  geometry_msgs::msg::Pose pose4 = pose3;
  pose4.position.x -= 0.1;
  pose4.position.y += 0.2;
  waypoints.push_back(pose4);
  
  // Compute Cartesian path
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  
  if (fraction > 0.9) {
    RCLCPP_INFO(logger, "Path computed successfully, executing...");
    move_group.execute(trajectory);
  } else {
    RCLCPP_ERROR(logger, "Path planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
