#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>

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

  auto qos = rclcpp::QoS(1).transient_local();
  auto scene_pub = node->create_publisher<moveit_msgs::msg::PlanningScene>("/planning_scene", qos);
  auto state_pub = node->create_publisher<moveit_msgs::msg::DisplayRobotState>("/display_robot_state", qos);

  YAML::Node config;
  try {
    config = YAML::LoadFile("benchmark_queries.yaml");
    RCLCPP_INFO(node->get_logger(), "Loaded YAML configuration.");
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Error reading YAML!");
    return 1;
  }

  moveit_msgs::msg::PlanningScene scene_msg;
  scene_msg.is_diff = true; 

  // --- ŁADOWANIE PRZESZKÓD ---
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

  // Publikujemy scenę pierwszy raz
  scene_pub->publish(scene_msg);

  YAML::Node queries = config["queries"];
  int num_queries = queries ? queries.size() : 0;
  
  if (num_queries == 0) {
    RCLCPP_WARN(node->get_logger(), "Nie znaleziono zapytań (queries) w pliku YAML!");
    return 0;
  }

  // --- FUNKCJA POMOCNICZA DO PUBLIKOWANIA STANU ---
  auto publish_state = [&](int index, bool is_start) {
    if (index < 0 || index >= num_queries) return;
    
    auto query_item = queries[index];
    std::string q_name = query_item.begin()->first.as<std::string>();
    YAML::Node q_data = query_item.begin()->second;

    auto joints = is_start ? q_data["start"].as<std::vector<double>>() : q_data["goal"].as<std::vector<double>>();

    moveit::core::RobotState rs(robot_model);
    rs.setJointGroupPositions(jmg, joints);
    
    moveit_msgs::msg::DisplayRobotState disp_msg;
    moveit::core::robotStateToRobotStateMsg(rs, disp_msg.state);
    state_pub->publish(disp_msg);
    
    std::string state_type = is_start ? "START" : "CEL (END)";
    RCLCPP_INFO(node->get_logger(), "[%s] Wyświetlam: %s", q_name.c_str(), state_type.c_str());
  };

  // --- INTERFEJS TERMINALOWY ---
  int current_index = 0;
  std::cout << "\n====================================================\n";
  std::cout << " Wczytano " << num_queries << " par zapytań (indeksy od 0 do " << num_queries - 1 << ").\n";
  std::cout << " DOSTĘPNE KOMENDY:\n";
  std::cout << "   start - pokazuje config początkowy dla obecnego indeksu\n";
  std::cout << "   end   - pokazuje config docelowy dla obecnego indeksu\n";
  std::cout << "   <nr>  - wpisz numer (np. 3), aby przeskoczyć do innej pary\n";
  std::cout << "   q     - wyjście z programu\n";
  std::cout << "====================================================\n";

  // Na starcie pokazujemy pozycję startową zerowego zapytania
  publish_state(current_index, true);

  std::string input;
  while (rclcpp::ok()) 
  {
    std::cout << "\n[Index: " << current_index << "] Komenda > " << std::flush;;
    if (!std::getline(std::cin, input)) break; // Wychwytuje np. Ctrl+D

    // Czyszczenie wejścia (usunięcie zbędnych spacji jeśli ktoś wpisze " start")
    input.erase(0, input.find_first_not_of(" \t\r\n"));
    input.erase(input.find_last_not_of(" \t\r\n") + 1);

    if (input == "q" || input == "quit" || input == "exit") {
      break;
    } 
    else if (input == "start") {
      publish_state(current_index, true);
      scene_pub->publish(scene_msg); // Upewniamy się, że klocki nie znikną w RVizie
    } 
    else if (input == "end") {
      publish_state(current_index, false);
      scene_pub->publish(scene_msg);
    } 
    else if (!input.empty()) {
      // Próba parsowania liczby
      try {
        int new_index = std::stoi(input);
        if (new_index >= 0 && new_index < num_queries) {
          current_index = new_index;
          // Automatycznie pokazujemy 'start' nowo wybranego zapytania
          publish_state(current_index, true);
          scene_pub->publish(scene_msg);
        } else {
          std::cout << " [!] Podano indeks poza zakresem (0 - " << num_queries - 1 << ").\n";
        }
      } catch (const std::invalid_argument&) {
        std::cout << " [!] Nieznana komenda. Wpisz 'start', 'end', numer lub 'q'.\n";
      } catch (const std::out_of_range&) {
        std::cout << " [!] Podana liczba jest zbyt duża.\n";
      }
    }
  }

  RCLCPP_INFO(node->get_logger(), "Zamykanie wizualizatora...");
  rclcpp::shutdown();
  return 0;
}