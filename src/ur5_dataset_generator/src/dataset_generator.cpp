#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <ctime>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ur5_dataset_generator", node_options);

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  std::srand(std::time(nullptr));
  int num_obstacles = 4;
  RCLCPP_INFO(node->get_logger(), "Adding %d random obstacles to the scene...", num_obstacles);

  // Wektor przechowujący wygenerowane przeszkody do późniejszego zapisu
  std::vector<moveit_msgs::msg::CollisionObject> generated_obstacles;

  for (int i = 0; i < num_obstacles; ++i) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "box_" + std::to_string(i);

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {
      0.1 + (std::rand() % 20) / 100.0,
      0.1 + (std::rand() % 20) / 100.0,
      0.1 + (std::rand() % 40) / 100.0
    };

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.3 + (std::rand() % 50) / 100.0;
    box_pose.position.y = -0.5 + (std::rand() % 100) / 100.0;
    box_pose.position.z = 0.1 + (std::rand() % 50) / 100.0;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene.processCollisionObjectMsg(collision_object);
    generated_obstacles.push_back(collision_object); // Zapisujemy przeszkodę w pamięci
  }

  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("ur_manipulator");

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  int valid_states_found = 0;
  int target_dataset_size = 10; 
  std::vector<std::vector<double>> dataset;

  RCLCPP_INFO(node->get_logger(), "Starting random state generation...");

  while (valid_states_found < target_dataset_size * 2)
  {
    collision_result.clear();
    current_state.setToRandomPositions(joint_model_group);
    current_state.update();
    
    planning_scene.checkSelfCollision(collision_request, collision_result, current_state);

    if (!collision_result.collision)
    {
      std::vector<double> joint_values;
      current_state.copyJointGroupPositions(joint_model_group, joint_values);
      dataset.push_back(joint_values);
      
      RCLCPP_INFO(node->get_logger(), "Valid state found! (%d)", valid_states_found + 1);
      valid_states_found++;
    }
  }

  // Zapis do pliku YAML - rozszerzony o sekcję obstacles
  std::ofstream file("benchmark_queries.yaml");
  
  file << "obstacles:\n";
  for (const auto& obs : generated_obstacles) {
    file << "  - id: " << obs.id << "\n";
    file << "    dimensions: [" 
         << obs.primitives[0].dimensions[0] << ", "
         << obs.primitives[0].dimensions[1] << ", "
         << obs.primitives[0].dimensions[2] << "]\n";
    file << "    position: [" 
         << obs.primitive_poses[0].position.x << ", "
         << obs.primitive_poses[0].position.y << ", "
         << obs.primitive_poses[0].position.z << "]\n";
  }

  file << "queries:\n";
  for (size_t i = 0; i < dataset.size(); i += 2) {
      file << "  - query_" << (i / 2 + 1) << ":\n";
      file << "      start: [";
      for (size_t j = 0; j < dataset[i].size(); ++j) file << dataset[i][j] << (j == dataset[i].size()-1 ? "" : ", ");
      file << "]\n      goal: [";
      for (size_t j = 0; j < dataset[i+1].size(); ++j) file << dataset[i+1][j] << (j == dataset[i+1].size()-1 ? "" : ", ");
      file << "]\n";
  }
  file.close();

  RCLCPP_INFO(node->get_logger(), "Saved dataset with obstacles to benchmark_queries.yaml");
  rclcpp::shutdown();
  return 0;
}