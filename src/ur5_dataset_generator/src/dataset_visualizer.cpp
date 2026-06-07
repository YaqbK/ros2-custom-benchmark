#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("dataset_visualizer", node_options);

  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  auto robot_model = robot_model_loader.getModel();
  if (!robot_model) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load robot model!");
    return 1;
  }
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("ur_manipulator");

  // Dodajemy QoS (Quality of Service), żeby RViz lepiej łapał nasze wiadomości
  auto qos = rclcpp::QoS(1).transient_local();
  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", qos);
  auto state_pub = node->create_publisher<moveit_msgs::msg::DisplayRobotState>("/display_robot_state", qos);

  YAML::Node config;
  try {
    config = YAML::LoadFile("benchmark_queries.yaml");
    RCLCPP_INFO(node->get_logger(), "Loaded benchmark_queries.yaml");
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Error reading YAML!");
    return 1;
  }

  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true; 

  if (config["obstacles"]) {
    for (const auto& obs_node : config["obstacles"]) {
      moveit_msgs::msg::CollisionObject obj;
      obj.id = obs_node["id"].as<std::string>();
      obj.header.frame_id = robot_model->getModelFrame();
      
      shape_msgs::msg::SolidPrimitive primitive;
      std::string type_str = obs_node["type"] ? obs_node["type"].as<std::string>() : "BOX";
      
      if (type_str == "BOX") primitive.type = primitive.BOX;
      else if (type_str == "CYLINDER") primitive.type = primitive.CYLINDER;
      else if (type_str == "SPHERE") primitive.type = primitive.SPHERE;
      
      auto dims = obs_node["dimensions"].as<std::vector<double>>();
      primitive.dimensions.resize(dims.size());
      for (size_t k = 0; k < dims.size(); ++k) primitive.dimensions[k] = dims[k];
      
      geometry_msgs::msg::Pose pose;
      auto pos = obs_node["position"].as<std::vector<double>>();
      pose.position.x = pos[0]; pose.position.y = pos[1]; pose.position.z = pos[2];
      pose.orientation.w = 1.0;
      
      obj.primitives.push_back(primitive);
      obj.primitive_poses.push_back(pose);
      obj.operation = obj.ADD;
      scene_msg.world.collision_objects.push_back(obj);
    }
  }

  RCLCPP_INFO(node->get_logger(), "Starting infinite loop. Press Ctrl+C in terminal to stop.");

  // NIESKOŃCZONA PĘTLA - Program nie zniknie, dopóki go nie zamkniesz
  while (rclcpp::ok()) 
  {
    // Publikujemy przeszkody przy każdym obrocie, żeby RViz na pewno je widział
    scene_pub->publish(scene_msg);

    if (config["queries"]) {
      for (const auto& query_item : config["queries"]) {
        if (!rclcpp::ok()) break; // Przerywamy, jeśli wciśnięto Ctrl+C

        std::string q_name = query_item.begin()->first.as<std::string>();
        YAML::Node q_data = query_item.begin()->second;

        auto start_joints = q_data["start"].as<std::vector<double>>();
        auto goal_joints = q_data["goal"].as<std::vector<double>>();

        moveit::core::RobotState rs(robot_model);
        moveit_msgs::msg::DisplayRobotState disp_msg;

        // --- START ---
        rs.setJointGroupPositions(jmg, start_joints);
        moveit::core::robotStateToRobotStateMsg(rs, disp_msg.state);
        state_pub->publish(disp_msg);
        RCLCPP_INFO(node->get_logger(), "[%s] Showing START", q_name.c_str());
        std::this_thread::sleep_for(3s);

        if (!rclcpp::ok()) break;

        // --- CEL ---
        rs.setJointGroupPositions(jmg, goal_joints);
        moveit::core::robotStateToRobotStateMsg(rs, disp_msg.state);
        state_pub->publish(disp_msg);
        RCLCPP_INFO(node->get_logger(), "[%s] Showing GOAL", q_name.c_str());
        std::this_thread::sleep_for(4s);
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}