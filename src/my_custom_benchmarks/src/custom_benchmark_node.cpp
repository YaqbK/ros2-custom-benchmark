#include <rclcpp/rclcpp.hpp>
#include <moveit/benchmarks/BenchmarkExecutor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h> 
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <map>
#include <string>
#include <vector>

class CustomBenchmarkOptions : public moveit_ros_benchmarks::BenchmarkOptions
{
public:
  CustomBenchmarkOptions(const rclcpp::Node::SharedPtr& node) : BenchmarkOptions(node)
  {
    this->planning_pipelines_["ompl"] = {
      "RRTConnectkConfigDefault", 
      "PRMkConfigDefault", 
      "RRTstarkConfigDefault",
      "AnytimePathShortening"
    };
    
    this->planning_pipelines_["chomp"] = {"CHOMP"};
  }

  const std::map<std::string, std::vector<std::string>>& getPlanningPipelinesMap() const
  {
    return planning_pipelines_;
  }
};

class CustomBenchmarkExecutor : public moveit_ros_benchmarks::BenchmarkExecutor
{
private:
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

    auto set_str_param = [&](const std::string& name, const std::string& val) {
      if (!node->has_parameter(name)) node->declare_parameter(name, val);
      else node->set_parameter(rclcpp::Parameter(name, val));
    };
    
    set_str_param("benchmark_config.parameters.query_regex", ".*");
    set_str_param("benchmark_config.parameters.start_state_regex", ".*");
    set_str_param("benchmark_config.parameters.goal_constraint_regex", ".*");
    set_str_param("benchmark_config.parameters.path_constraint_regex", ".*");
    set_str_param("benchmark_config.parameters.trajectory_constraint_regex", ".*");

    // Wymuszenie ładowania trzech stabilnych potoków
    std::vector<std::string> target_pipelines = {"ompl", "chomp", "pilz_industrial_motion_planner"};
    if (!node->has_parameter("benchmark_config.planning_pipelines.pipelines")) {
      node->declare_parameter("benchmark_config.planning_pipelines.pipelines", target_pipelines);
    } else {
      node->set_parameter(rclcpp::Parameter("benchmark_config.planning_pipelines.pipelines", target_pipelines));
    }

    CustomBenchmarkOptions options(node);

    auto ompl_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "ompl");
    this->planning_pipelines_["ompl"] = ompl_pipeline;

    auto chomp_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "chomp");
    this->planning_pipelines_["chomp"] = chomp_pipeline;

    auto pilz_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "pilz_industrial_motion_planner");
    this->planning_pipelines_["pilz_industrial_motion_planner"] = pilz_pipeline;

    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    
    YAML::Node config;
    try {
      config = YAML::LoadFile("benchmark_queries.yaml");
      RCLCPP_INFO(node->get_logger(), "Successfully loaded benchmark_queries.yaml");
    } catch (const YAML::Exception& e) {
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
        scene->processCollisionObjectMsg(obj);
      }
      RCLCPP_INFO(node->get_logger(), "Loaded %ld multi-type obstacles.", config["obstacles"].size());
    }

    moveit_msgs::msg::PlanningScene scene_msg;
    scene->getPlanningSceneMsg(scene_msg);

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
          request.num_planning_attempts = 1; 
          request.allowed_planning_time = yaml_timeout;

          request.max_velocity_scaling_factor = 1.0;
          request.max_acceleration_scaling_factor = 1.0;
          
          moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
          request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg));

          BenchmarkRequest benchmark_query;
          benchmark_query.name = q_name;
          benchmark_query.request = request;
          
          preloaded_queries_.push_back(benchmark_query);
        }
      }
      RCLCPP_INFO(node->get_logger(), "Loaded %ld queries to preloaded cache.", preloaded_queries_.size());
    }

    RCLCPP_INFO(node->get_logger(), "[*] Rozpoczynam analize i budowanie macierzy bledow dla plannerow...");
    
    std::map<std::string, planning_pipeline::PlanningPipelinePtr> local_pipelines;
    local_pipelines["ompl"] = ompl_pipeline;
    local_pipelines["chomp"] = chomp_pipeline;
    local_pipelines["pilz_industrial_motion_planner"] = pilz_pipeline;

    std::vector<YAML::Node> failed_queries_list;
    bool system_has_failures = false;

    for (const auto& query : preloaded_queries_) {
      bool current_query_failed_somewhere = false;
      YAML::Node planners_status_node;

      for (const auto& pipeline_entry : options.getPlanningPipelinesMap()) {
        std::string pipeline_id = pipeline_entry.first;
        auto pipeline_ptr = local_pipelines[pipeline_id];
        
        if (!pipeline_ptr) continue;

        for (const std::string& planner_id : pipeline_entry.second) {
          moveit_msgs::msg::MotionPlanRequest test_req = query.request;
          test_req.planner_id = planner_id;
          test_req.pipeline_id = pipeline_id;

          planning_interface::MotionPlanResponse test_res;
          bool success = pipeline_ptr->generatePlan(scene, test_req, test_res);
          
          std::string outcome = success ? "SUCCESS" : "FAILED";
          planners_status_node[pipeline_id + "_" + planner_id] = outcome;
          
          if (!success) {
            current_query_failed_somewhere = true;
          }
        }
      }

      if (current_query_failed_somewhere) {
        system_has_failures = true;
        YAML::Node failed_node;
        YAML::Node failed_data;

        for (const auto& orig_query : config["queries"]) {
          if (orig_query[query.name]) {
            failed_data["start"] = orig_query[query.name]["start"];
            failed_data["goal"] = orig_query[query.name]["goal"];
            break;
          }
        }
        failed_data["planners_status"] = planners_status_node;
        failed_node[query.name] = failed_data;
        failed_queries_list.push_back(failed_node);
      }
    }

    if (system_has_failures) {
      YAML::Node failed_yaml_root;
      if (config["obstacles"]) {
        failed_yaml_root["obstacles"] = config["obstacles"];
      }
      failed_yaml_root["queries"] = failed_queries_list;

      std::ofstream failed_file("failed_queries.yaml");
      failed_file << failed_yaml_root;
      failed_file.close();
      RCLCPP_WARN(node->get_logger(), "[!] Znaleziono bledy! Macierz porazek zapisana do pliku: failed_queries.yaml");
    } else {
      RCLCPP_INFO(node->get_logger(), "[+] Wszystkie plannery poradzily sobie idealnie ze wszystkimi zadaniami.");
    }

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