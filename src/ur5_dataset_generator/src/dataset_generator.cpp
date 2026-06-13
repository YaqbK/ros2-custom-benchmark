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
#include <cmath>

// Pomocnicza funkcja do losowania wartości z zakresu
double randomDouble(double min, double max) {
    return min + (max - min) * ((double)std::rand() / RAND_MAX);
}

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

  // =====================================================================
  // --- KONFIGURACJA PARAMETRÓW SCENY ---
  // =====================================================================
  int min_obstacles = 5;
  int max_obstacles = 8;
  int target_dataset_size = 250;

  double ws_z_min = -0.8,  ws_z_max = 0.8;

  // Rozmiary brył (min i max dla promieni, wysokości i krawędzi)
  double size_min = 0.05, size_max = 0.25;
  // =====================================================================

  int num_obstacles = min_obstacles + (std::rand() % (max_obstacles - min_obstacles + 1));
  RCLCPP_INFO(node->get_logger(), "Generating %d random obstacles...", num_obstacles);

  std::vector<moveit_msgs::msg::CollisionObject> generated_obstacles;

  for (int i = 0; i < num_obstacles; ++i) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "obs_" + std::to_string(i);

    shape_msgs::msg::SolidPrimitive primitive;
    int shape_type = std::rand() % 3; // 0: BOX, 1: CYLINDER, 2: SPHERE

    if (shape_type == 0) {
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {randomDouble(size_min, size_max), randomDouble(size_min, size_max), randomDouble(size_min, size_max)};
    } else if (shape_type == 1) {
        primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        primitive.dimensions = {randomDouble(size_min, size_max * 1.5), randomDouble(size_min, size_max / 2)}; // HEIGHT, RADIUS
    } else {
        primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        primitive.dimensions = {randomDouble(size_min, size_max / 1.5)}; // RADIUS
    }

    // Promień: od 0.35m (bezpiecznie poza podstawą) do 0.85m (zasięg ramienia UR5e)
    double r_min = 0.35;
    double r_max = 0.85;
    double r = randomDouble(r_min, r_max);
    
    // Pełne 360 stopni dookoła robota
    double theta = randomDouble(0.0, 2.0 * M_PI); 

    geometry_msgs::msg::Pose pose;
    pose.position.x = r * std::cos(theta);
    pose.position.y = r * std::sin(theta);
    pose.position.z = randomDouble(ws_z_min, ws_z_max);
    pose.orientation.w = 1.0;
    
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    planning_scene.processCollisionObjectMsg(collision_object);
    generated_obstacles.push_back(collision_object); 
  }

  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("ur_manipulator");

  collision_detection::CollisionRequest collision_request;  
  collision_detection::CollisionResult collision_result;

  int valid_states_found = 0;
  std::vector<std::vector<double>> dataset;

  while (valid_states_found < target_dataset_size * 2)
  {
    collision_result.clear();
    current_state.setToRandomPositions(joint_model_group);
    current_state.update();
    
    planning_scene.checkCollision(collision_request, collision_result, current_state);

    if (!collision_result.collision)
    {
      std::vector<double> joint_values;
      current_state.copyJointGroupPositions(joint_model_group, joint_values);
      dataset.push_back(joint_values);
      
      RCLCPP_INFO(node->get_logger(), "Valid state found! (%d / %d)", valid_states_found + 1, target_dataset_size * 2);
      valid_states_found++;
    }
  }

  // Zapis do pliku YAML
  // =========================================================
  // 1. Zapis samych przeszkód do pliku obstacles.yaml
  // =========================================================
  std::ofstream obs_file("obstacles.yaml");
  obs_file << "obstacles:\n";
  for (const auto& obs : generated_obstacles) {
    std::string type_str;
    switch(obs.primitives[0].type) {
        case shape_msgs::msg::SolidPrimitive::BOX: type_str = "BOX"; break;
        case shape_msgs::msg::SolidPrimitive::CYLINDER: type_str = "CYLINDER"; break;
        case shape_msgs::msg::SolidPrimitive::SPHERE: type_str = "SPHERE"; break;
    }
    
    obs_file << "  - id: " << obs.id << "\n";
    obs_file << "    type: " << type_str << "\n";
    obs_file << "    dimensions: [";
    for (size_t k = 0; k < obs.primitives[0].dimensions.size(); ++k) {
        obs_file << obs.primitives[0].dimensions[k];
        if (k < obs.primitives[0].dimensions.size() - 1) obs_file << ", ";
    }
    obs_file << "]\n    position: [" 
             << obs.primitive_poses[0].position.x << ", "
             << obs.primitive_poses[0].position.y << ", "
             << obs.primitive_poses[0].position.z << "]\n";
  }
  obs_file.close();
  RCLCPP_INFO(node->get_logger(), "Saved obstacles to obstacles.yaml");

  // =========================================================
  // 2. Zapis samych zapytań (start/goal) do pliku queries.yaml
  // =========================================================
  std::ofstream queries_file("queries.yaml");
  queries_file << "queries:\n";
  for (size_t i = 0; i < dataset.size(); i += 2) {
      queries_file << "  - query_" << (i / 2 + 1) << ":\n";
      queries_file << "      start: [";
      for (size_t j = 0; j < dataset[i].size(); ++j) queries_file << dataset[i][j] << (j == dataset[i].size()-1 ? "" : ", ");
      queries_file << "]\n      goal: [";
      for (size_t j = 0; j < dataset[i+1].size(); ++j) queries_file << dataset[i+1][j] << (j == dataset[i+1].size()-1 ? "" : ", ");
      queries_file << "]\n";
  }
  queries_file.close();
  RCLCPP_INFO(node->get_logger(), "Saved queries to queries.yaml");

  rclcpp::shutdown();
  return 0;
}