#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <fstream>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("ur5_dataset_generator", node_options);

  // 1. Setup the Planning Scene (exactly as the tutorial shows)
  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  // 2. Setup the Robot State
  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("ur_manipulator");

  // 3. Collision Request Setup
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  int valid_states_found = 0;
  int target_dataset_size = 10; // 10 pairs (Start + Goal) = 20 states
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

  // Save the dataset to a YAML file
  std::ofstream file("benchmark_queries.yaml");
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

  RCLCPP_INFO(node->get_logger(), "Saved dataset to benchmark_queries.yaml");
  rclcpp::shutdown();
  return 0;
}