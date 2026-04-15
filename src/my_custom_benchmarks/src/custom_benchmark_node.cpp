#include <rclcpp/rclcpp.hpp>
#include <moveit/benchmarks/BenchmarkExecutor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

class CustomBenchmarkOptions : public moveit_ros_benchmarks::BenchmarkOptions
{
public:
  CustomBenchmarkOptions(const rclcpp::Node::SharedPtr& node) : BenchmarkOptions(node)
  {
    // Inheritance gives us VIP access to the protected variable!
    this->planning_pipelines_["ompl"] = {
      "RRTConnectkConfigDefault", 
      "PRMkConfigDefault", 
      "RRTstarkConfigDefault"
    };
  }
};

class CustomBenchmarkExecutor : public moveit_ros_benchmarks::BenchmarkExecutor
{
protected:
  bool loadBenchmarkQueryData(
      const moveit_ros_benchmarks::BenchmarkOptions& options,
      moveit_msgs::msg::PlanningScene& scene_msg,
      std::vector<StartState>& start_states,
      std::vector<PathConstraints>& path_constraints,
      std::vector<PathConstraints>& goal_constraints,
      std::vector<TrajectoryConstraints>& traj_constraints,
      std::vector<BenchmarkRequest>& queries) override
  {
    // 1. Create a new benchmark request container using the local struct
    BenchmarkRequest req;
    req.name = "ur5e_custom_query";
    
    // 2. Set the core parameters
    req.request.group_name = "ur_manipulator";
    req.request.allowed_planning_time = 5.0;
    req.request.num_planning_attempts = 10;
    
    // 3. Set Start State (make sure scene_msg in the parameters above is not commented out!)
    req.request.start_state = scene_msg.robot_state;
    
    // 4. Set Goal State: Move the shoulder pan joint to ~90 degrees (1.57 radians)
    moveit_msgs::msg::Constraints goal_constraint;
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = "shoulder_pan_joint";
    joint_constraint.position = 1.57; 
    joint_constraint.tolerance_above = 0.01;
    joint_constraint.tolerance_below = 0.01;
    joint_constraint.weight = 1.0;
    
    goal_constraint.joint_constraints.push_back(joint_constraint);
    req.request.goal_constraints.push_back(goal_constraint);
    
    // 5. Push our custom request into MoveIt's query queue (the 7th parameter)
    queries.push_back(req);
    
    return true; 
  }

public:
  CustomBenchmarkExecutor(const rclcpp::Node::SharedPtr& node) : BenchmarkExecutor(node) {}

  // This function is INSIDE the class, so it CAN access protected BenchmarkRequest and initializeBenchmarks
  void runCustomBenchmark(const rclcpp::Node::SharedPtr& node)
  {
    // 1. Load Robot Model
    robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
    auto robot_model = robot_model_loader.getModel();
    if (!robot_model) {
      RCLCPP_ERROR(node->get_logger(), "Could not load robot model!");
      return;
    }

    // 2. Setup Options
    CustomBenchmarkOptions options(node);

    // BYPASS THE EXECUTOR BUG: Manually build the pipeline and inject it!
    auto pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node, "ompl");
    this->planning_pipelines_["ompl"] = pipeline;

    // 3. Create Planning Scene
    planning_scene::PlanningScene scene(robot_model);
    moveit_msgs::msg::CollisionObject box;
    box.id = "test_obstacle";
    box.header.frame_id = robot_model->getModelFrame();
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.2, 0.8, 0.5}; 
    
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.5; 
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;
    box_pose.orientation.w = 1.0;
    
    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;
    scene.processCollisionObjectMsg(box);

    moveit_msgs::msg::PlanningScene scene_msg;
    scene.getPlanningSceneMsg(scene_msg);

    // 4. Setup Queries using the PROTECTED BenchmarkRequest struct
    std::vector<BenchmarkRequest> queries;
    
    moveit::core::RobotState start_state(robot_model);
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("ur_manipulator");
    
    std::vector<double> start_joints = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
    start_state.setJointGroupPositions(jmg, start_joints);
    
    moveit::core::RobotState goal_state(start_state);
    std::vector<double> goal_joints = {1.57, -1.57, 1.57, -1.57, -1.57, 0.0};
    goal_state.setJointGroupPositions(jmg, goal_joints);

    moveit_msgs::msg::MotionPlanRequest request;
    request.group_name = "ur_manipulator";
    request.num_planning_attempts = 10;
    request.allowed_planning_time = 5.0;
    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
    request.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(goal_state, jmg));

    // Fill the protected struct
    BenchmarkRequest benchmark_query;
    benchmark_query.name = "ur5e_obstacle_avoidance";
    benchmark_query.request = request;
    queries.push_back(benchmark_query);

    // 5. Run using the PROTECTED methods
    if (this->initializeBenchmarks(options, scene_msg, queries))
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

  // Simply create the object and call our "unlocked" method
  CustomBenchmarkExecutor executor(node);
  executor.runCustomBenchmark(node);

  rclcpp::shutdown();
  return 0;
}
