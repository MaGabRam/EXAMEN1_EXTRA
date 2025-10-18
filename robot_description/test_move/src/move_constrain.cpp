#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("add_box_to_rviz2");
  auto logger = rclcpp::get_logger("add_box_to_rviz2");

  // Create a PlanningSceneInterface to manage collision objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Define a collision object
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world"; // Change as needed
  collision_object.id = "box1";

  // Define a box shape
  shape_msgs::msg::SolidPrimitive box_primitive;
  box_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  box_primitive.dimensions = {0.4, 0.2, 0.2}; // x, y, z dimensions

  // Define the box's pose
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.1;

  // Assign shape and pose to the collision object
  collision_object.primitives.push_back(box_primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Publish the collision object to the planning scene
  planning_scene_interface.applyCollisionObject(collision_object);

  RCLCPP_INFO(logger, "Added a box to the planning scene.");

  // Keep node alive for visualization
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
