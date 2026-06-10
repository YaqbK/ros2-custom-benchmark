#include <rclcpp/rclcpp.hpp>
#include <moveit/benchmarks/BenchmarkExecutor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

class CustomBenchmarkOptions : public moveit_ros_benchmarks::BenchmarkOptions
{
public:
  // tutaj dodajemy planery które chcemy przetestować
  CustomBenchmarkOptions(const rclcpp::Node::SharedPtr& node) : BenchmarkOptions(node)
  {
    this->planning_pipelines_["ompl"] = {
      "RRTConnectkConfigDefault", 
      "PRMkConfigDefault", 
      "RRTstarkConfigDefault"
    };
    this->planning_pipelines_["chomp"] = {"CHOMP"};
  }
};

class CustomBenchmarkExecutor : public moveit_ros_benchmarks::BenchmarkExecutor
{
private:
  // chowamy zapytania z pliku YAML
  std::vector<BenchmarkRequest> preloaded_queries_;

protected:
  bool loadBenchmarkQueryData(
      const moveit_ros_benchmarks::BenchmarkOptions& /*options*/,
      moveit_msgs::msg::PlanningScene& /*scene_msg*/,
      std::vector<StartState>& /*start_states*/,
      std::vector<PathConstraints>& /*path_constraints*/,
      std::vector<PathConstraints>& /*goal_constraints*/,
      std::vector<TrajectoryConstraints>& /*traj_constraints*/,
      std::vector<BenchmarkRequest>& queries) override
  {
    queries = preloaded_queries_;
    return true; 
  }

public:
  CustomBenchmarkExecutor(const rclcpp::Node::SharedPtr& node) : BenchmarkExecutor(node) {}

  void runCustomBenchmark(const rclcpp::Node::SharedPtr& node)
  {
    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    auto robot_model = robot_model_loader.getModel();
    if (!robot_model) {
      RCLCPP_ERROR(node->get_logger(), "Could not load robot model!");
      return;
    }

    // --- WSTRZYKNIĘCIE PARAMETRÓW REGEX Z ".parameters." ---
    auto set_str_param = [&](const std::string& name, const std::string& val) {
      if (!node->has_parameter(name)) node->declare_parameter(name, val);
      else node->set_parameter(rclcpp::Parameter(name, val));
    };
    
    // Wymuszamy parametry tak, jak szuka ich parser
    set_str_param("benchmark_config.parameters.query_regex", ".*");
    set_str_param("benchmark_config.parameters.start_state_regex", ".*");
    set_str_param("benchmark_config.parameters.goal_constraint_regex", ".*");
    set_str_param("benchmark_config.parameters.path_constraint_regex", ".*");
    set_str_param("benchmark_config.parameters.trajectory_constraint_regex", ".*");

    if (!node->has_parameter("benchmark_config.planning_pipelines.pipelines")) {
      node->declare_parameter("benchmark_config.planning_pipelines.pipelines", std::vector<std::string>{"ompl"});
    } else {
      node->set_parameter(rclcpp::Parameter("benchmark_config.planning_pipelines.pipelines", std::vector<std::string>{"ompl"}));
    }
    // -------------------------------------------------------

    CustomBenchmarkOptions options(node);

    // Ładowanie wszystkich planning pipelines
    auto ompl_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "ompl");
    this->planning_pipelines_["ompl"] = ompl_pipeline;

    auto chomp_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "chomp");
    this->planning_pipelines_["chomp"] = chomp_pipeline;

    planning_scene::PlanningScene scene(robot_model);
    
    YAML::Node config;
    try {
      config = YAML::LoadFile("benchmark_queries.yaml");
      RCLCPP_INFO(node->get_logger(), "Successfully loaded benchmark_queries.yaml");
    } catch (const YAML::BadFile& e) {
      RCLCPP_ERROR(node->get_logger(), "Failed to load benchmark_queries.yaml!");
      return;
    }

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
        
        // Zabezpieczenie przed błędem zapytań (przypisujemy wektor dynamicznie)
        auto dims = obs_node["dimensions"].as<std::vector<double>>();
        primitive.dimensions.resize(dims.size());
        for (size_t k = 0; k < dims.size(); ++k) {
            primitive.dimensions[k] = dims[k];
        }
        
        geometry_msgs::msg::Pose pose;
        auto pos = obs_node["position"].as<std::vector<double>>();
        pose.position.x = pos[0];
        pose.position.y = pos[1];
        pose.position.z = pos[2];
        pose.orientation.w = 1.0;
        
        obj.primitives.push_back(primitive);
        obj.primitive_poses.push_back(pose);
        obj.operation = obj.ADD;
        scene.processCollisionObjectMsg(obj);
      }
      RCLCPP_INFO(node->get_logger(), "Loaded %ld multi-type obstacles.", config["obstacles"].size());
    }

    moveit_msgs::msg::PlanningScene scene_msg;
    scene.getPlanningSceneMsg(scene_msg);

    // Czyszczenie skrytki
    preloaded_queries_.clear(); 
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("ur_manipulator");

    double yaml_timeout = 10.0;
    node->get_parameter_or("benchmark_config.parameters.timeout", yaml_timeout, 10.0);

    if (config["queries"]) {
      for (const auto& query_item : config["queries"]) {
        for (auto it = query_item.begin(); it != query_item.end(); ++it) {
          std::string q_name = it->first.as<std::string>();
          YAML::Node q_data = it->second;

          auto start_joints = q_data["start"].as<std::vector<double>>();
          auto goal_joints = q_data["goal"].as<std::vector<double>>();

          moveit::core::RobotState start_state(robot_model);
          start_state.setJointGroupPositions(jmg, start_joints);
          start_state.update();

          moveit::core::RobotState goal_state(robot_model);
          goal_state.setJointGroupPositions(jmg, goal_joints);
          goal_state.update();

          moveit_msgs::msg::MotionPlanRequest request;
          request.group_name = "ur_manipulator";
          request.num_planning_attempts = 10;
          request.allowed_planning_time = yaml_timeout;
          
          moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
          request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg));

          BenchmarkRequest benchmark_query;
          benchmark_query.name = q_name;
          benchmark_query.request = request;
          
          // ŁADUJEMY ZAPYTANIE DO NASZEJ BEZPIECZNEJ SKRYTKI
          preloaded_queries_.push_back(benchmark_query);
        }
      }
      RCLCPP_INFO(node->get_logger(), "Loaded %ld queries to preloaded cache.", preloaded_queries_.size());
    }

    // Pusty wektor (i tak zostanie wewnętrznie wyczyszczony przez MoveIt i zastąpiony naszą funkcją)
    std::vector<BenchmarkRequest> dummy_queries;
    if (this->initializeBenchmarks(options, scene_msg, dummy_queries))
    {
      this->runBenchmarks(options);
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("custom_benchmark_node", node_options);

  CustomBenchmarkExecutor executor(node);
  executor.runCustomBenchmark(node);

  rclcpp::shutdown();
  return 0;
}