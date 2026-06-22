#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <cmath>
#include <map>
#include <algorithm>
#include <atomic>
#include <csignal>

// Include Eigen for mathematical transformations
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "zenpool_draw_letter/font_library.h"

// ==============================================================================
// SIGNAL HANDLING FOR AUTOMATIC CLEANUP
// ==============================================================================
std::atomic<bool> g_abort{false};
moveit::planning_interface::MoveGroupInterface* g_mgi = nullptr;

void sigint_handler(int /*sig*/)
{
  g_abort = true; // Signal all functions to skip to the end
  if (g_mgi) {
    g_mgi->stop(); // Physically halt the robot arm immediately
  }
  // We intentionally do NOT call rclcpp::shutdown() here!
  // We leave ROS alive so the main thread can successfully broadcast the "remove collision objects" message.
}

class Tool
{
public:
  Tool(const std::string& id,
       const std::string& mesh_resource,
       const std::string& ee_link,
       const geometry_msgs::msg::Pose& pose)
  : id_(id), ee_link_(ee_link)
  {
    shapes::Mesh* m = shapes::createMeshFromResource(mesh_resource);
    shapes::ShapeMsg mesh_msg_tmp;
    shapes::constructMsgFromShape(m, mesh_msg_tmp);
    mesh_msg_ = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);
    pose_ = pose;
  }

  void attach(moveit::planning_interface::PlanningSceneInterface& psi)
  {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = id_;
    collision_object.header.frame_id = ee_link_;
    collision_object.meshes.push_back(mesh_msg_);
    collision_object.mesh_poses.push_back(pose_);
    collision_object.operation = collision_object.ADD;

    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = ee_link_;
    attached_object.object = collision_object;
    attached_object.touch_links = { ee_link_, "tc_arm_side" };

    psi.applyAttachedCollisionObject(attached_object);
  }

  void detach(moveit::planning_interface::PlanningSceneInterface& psi, bool remove_from_world = true)
  {
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.object.id = id_;
    detach_object.link_name = ee_link_;
    detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyAttachedCollisionObject(detach_object);

    if (remove_from_world) {
      psi.removeCollisionObjects({ id_ });
    }
  }

private:
  std::string id_;
  std::string ee_link_;
  shape_msgs::msg::Mesh mesh_msg_;
  geometry_msgs::msg::Pose pose_;
};

// ==============================================================================
// CORE EXECUTION FUNCTIONS
// ==============================================================================

std::vector<double> makeJointTarget(const std::vector<double>& deg_values) {
  std::vector<double> rad_values(deg_values.size());
  for (size_t i = 0; i < deg_values.size(); ++i) {
    rad_values[i] = deg_values[i] * M_PI / 180.0;
  }
  return rad_values;
}

bool executeJoint(moveit::planning_interface::MoveGroupInterface& mgi, const std::vector<double>& joint_positions,
                  const rclcpp::Logger& logger, auto& draw_traj, auto& mvt, auto& prompt, const std::string& label) {
  if (g_abort) return false;
  
  mgi.setJointValueTarget(joint_positions);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(mgi.plan(plan))) {
    RCLCPP_ERROR(logger, "Planning failed for %s", label.c_str());
    return false;
  }
  
  draw_traj(plan.trajectory_);
  mvt.trigger();
  prompt("Execute " + label);
  
  if (g_abort) return false;
  mgi.execute(plan);
  return true;
}

bool executeCartesianWaypoints(moveit::planning_interface::MoveGroupInterface& mgi,
                               const std::vector<geometry_msgs::msg::Pose>& waypoints,
                               const rclcpp::Logger& logger, auto& draw_traj, auto& mvt, auto& prompt,
                               const std::string& label, float v_scale = 0.1, float a_scale = 0.1) {
  if (g_abort) return false;

  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mgi.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

  if (fraction < 0.9) {
    RCLCPP_WARN(logger, "Cartesian path incomplete for %s (%.2f%%)", label.c_str(), fraction * 100.0);
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  robot_trajectory::RobotTrajectory rt(mgi.getRobotModel(), mgi.getName());
  rt.setRobotTrajectoryMsg(*mgi.getCurrentState(), plan.trajectory_);
  
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  totg.computeTimeStamps(rt, v_scale, a_scale);
  rt.getRobotTrajectoryMsg(plan.trajectory_);

  draw_traj(plan.trajectory_);
  mvt.trigger();
  prompt("Execute " + label);
  
  if (g_abort) return false;
  mgi.execute(plan);
  return true;
}

bool executeCartesian(moveit::planning_interface::MoveGroupInterface& mgi,
                      const std::vector<double>& target_joint_positions,
                      const rclcpp::Logger& logger, auto& draw_traj, auto& mvt, auto& prompt,
                      const std::string& label, float v_scale = 0.1, float a_scale = 0.1) {
  if (g_abort) return false;

  geometry_msgs::msg::Pose start_pose = mgi.getCurrentPose().pose;
  moveit::core::RobotStatePtr k_state = std::make_shared<moveit::core::RobotState>(*mgi.getCurrentState());
  k_state->setJointGroupPositions(k_state->getJointModelGroup(mgi.getName()), target_joint_positions);
  const Eigen::Isometry3d& end_state = k_state->getGlobalLinkTransform(mgi.getEndEffectorLink());

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = end_state.translation().x();
  target_pose.position.y = end_state.translation().y();
  target_pose.position.z = end_state.translation().z();
  Eigen::Quaterniond q(end_state.rotation());
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();

  return executeCartesianWaypoints(mgi, {start_pose, target_pose}, logger, draw_traj, mvt, prompt, label, v_scale, a_scale);
}

bool executeNamed(moveit::planning_interface::MoveGroupInterface& mgi, const std::string& name,
                  const rclcpp::Logger& logger, auto& draw_traj, auto& mvt, auto& prompt, const std::string& label) {
  if (g_abort) return false;

  mgi.setNamedTarget(name);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(mgi.plan(plan))) return false;
  draw_traj(plan.trajectory_);
  mvt.trigger();
  prompt(label);
  
  if (g_abort) return false;
  mgi.execute(plan);
  return true;
}

// ==============================================================================
// LOGICAL ABSTRACTIONS (Sweep & Trace wrappers)
// ==============================================================================

void executeCleanSweep(moveit::planning_interface::MoveGroupInterface& mgi,
                       const rclcpp::Logger& logger, auto& draw_traj, auto& mvt, auto& prompt,
                       float v_scale, float a_scale) 
{
  if (g_abort) return;

  moveit::core::RobotStatePtr current_state = mgi.getCurrentState();
  if (!current_state->knowsFrameTransform("pool")) {
      RCLCPP_ERROR(logger, "CRITICAL: 'pool' link is missing from URDF!");
      return;
  }
  
  const Eigen::Isometry3d& T_base_pool = current_state->getGlobalLinkTransform("pool");

  double rot_x = M_PI;  
  double rot_y = 0.0;   
  double rot_z = 0.0;  

  Eigen::Matrix3d R_cleaner_in_pool = (Eigen::AngleAxisd(rot_x, Eigen::Vector3d::UnitX())
                                     * Eigen::AngleAxisd(rot_y, Eigen::Vector3d::UnitY())
                                     * Eigen::AngleAxisd(rot_z, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  Eigen::Quaterniond q_cleaner_base(T_base_pool.rotation() * R_cleaner_in_pool);

  geometry_msgs::msg::Quaternion cleaner_orient;
  cleaner_orient.x = q_cleaner_base.x();
  cleaner_orient.y = q_cleaner_base.y(); 
  cleaner_orient.z = q_cleaner_base.z();
  cleaner_orient.w = q_cleaner_base.w();

  double constant_z = -0.115 + 0.115 + 0.02 - 0.019; // Base Math - Depth
  Eigen::Vector3d sweep_start_point(-0.5, -0.35, constant_z); 
  Eigen::Vector3d sweep_end_point(0.5, -0.6, constant_z); 

  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto create_pose = [&](const Eigen::Vector3d& pos) {
      geometry_msgs::msg::Pose p; p.position.x = pos.x(); p.position.y = pos.y(); p.position.z = pos.z(); p.orientation = cleaner_orient; return p;
  };

  int num_sweeps = 4;
  if (num_sweeps % 2 != 0) num_sweeps++; 
  double step_y = (sweep_end_point.y() - sweep_start_point.y()) / std::max(1, (num_sweeps - 1));

  waypoints.push_back(create_pose(T_base_pool * (sweep_start_point + Eigen::Vector3d(0, 0, 0.05))));
  waypoints.push_back(create_pose(T_base_pool * sweep_start_point));

  for (int i = 0; i < num_sweeps; ++i) {
      double current_y = sweep_start_point.y() + i * step_y;
      double target_x = (i % 2 == 0) ? sweep_end_point.x() : sweep_start_point.x();
      waypoints.push_back(create_pose(T_base_pool * Eigen::Vector3d(target_x, current_y, sweep_start_point.z())));
      if (i < num_sweeps - 1) {
          waypoints.push_back(create_pose(T_base_pool * Eigen::Vector3d(target_x, sweep_start_point.y() + (i + 1) * step_y, sweep_start_point.z())));
      }
  }
  waypoints.push_back(create_pose(T_base_pool * (Eigen::Vector3d(sweep_start_point.x(), sweep_end_point.y(), sweep_start_point.z()) + Eigen::Vector3d(0, 0, 0.05))));

  if (g_abort) return;
  executeCartesianWaypoints(mgi, {waypoints.front()}, logger, draw_traj, mvt, prompt, "Approach Sweep", v_scale, a_scale);
  
  if (g_abort) return;
  executeCartesianWaypoints(mgi, waypoints, logger, draw_traj, mvt, prompt, "Execute Sweep Area", v_scale, a_scale);
}

void executeTextTracing(moveit::planning_interface::MoveGroupInterface& mgi, const std::string& text,
                        const rclcpp::Logger& logger, auto& draw_traj, auto& mvt, auto& prompt,
                        float v_scale, float a_scale) 
{
  if (g_abort) return;

  moveit::core::RobotStatePtr current_state = mgi.getCurrentState();
  if (!current_state->knowsFrameTransform("pool")) return;
  
  const Eigen::Isometry3d& T_base_pool = current_state->getGlobalLinkTransform("pool");
  Eigen::Quaterniond q_tool_base(T_base_pool.rotation() * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix());
  geometry_msgs::msg::Quaternion tool_orient;
  tool_orient.x = q_tool_base.x(); tool_orient.y = q_tool_base.y(); tool_orient.z = q_tool_base.z(); tool_orient.w = q_tool_base.w();

  Eigen::Vector3d dir_x = T_base_pool.linear() * Eigen::Vector3d(1,0,0);
  Eigen::Vector3d dir_y = T_base_pool.linear() * Eigen::Vector3d(0,1,0);
  Eigen::Vector3d dir_z = dir_x.cross(dir_y).normalized(); 

  Eigen::Vector3d origin_base = T_base_pool * Eigen::Vector3d(-0.4, -0.58, -0.115 + 0.115 + 0.02 - 0.03); 
  
  auto font = createFont();
  std::vector<std::string> chars = splitUTF8(text);
  
  double unscaled_width = 0.0;
  for (size_t i = 0; i < chars.size(); ++i) {
    unscaled_width += (font.count(chars[i]) ? font[chars[i]].width : 0.5);
    if (i < chars.size() - 1) unscaled_width += 0.35; // spacing
  }
  double scale = 0.8 / unscaled_width; // 0.8m total width
  double current_x_offset = 0.0;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto create_pose = [&](const Eigen::Vector3d& pos) {
      geometry_msgs::msg::Pose p; p.position.x = pos.x(); p.position.y = pos.y(); p.position.z = pos.z(); p.orientation = tool_orient; return p;
  };

  for (const std::string& c : chars) {
    if (font.count(c) == 0) { current_x_offset += (0.5 + 0.2) * scale; continue; }
    for (const auto& stroke : font[c].strokes) {
        Eigen::Vector3d start_pos = origin_base + dir_x * (current_x_offset + stroke.pts.front().x() * scale) + dir_y * (stroke.pts.front().y() * scale);
        waypoints.push_back(create_pose(start_pos + dir_z * 0.04));
        for (const auto& pt : stroke.pts) {
            waypoints.push_back(create_pose(origin_base + dir_x * (current_x_offset + pt.x() * scale) + dir_y * (pt.y() * scale)));
        }
        Eigen::Vector3d end_pos = origin_base + dir_x * (current_x_offset + stroke.pts.back().x() * scale) + dir_y * (stroke.pts.back().y() * scale);
        waypoints.push_back(create_pose(end_pos + dir_z * 0.04));
    }
    current_x_offset += (font[c].width + 0.2) * scale;
  }

  if (!waypoints.empty() && !g_abort) {
      executeCartesianWaypoints(mgi, {waypoints.front()}, logger, draw_traj, mvt, prompt, "Approach Text", v_scale, a_scale);
      if (!g_abort) {
        executeCartesianWaypoints(mgi, waypoints, logger, draw_traj, mvt, prompt, "Tracing " + text, v_scale, a_scale);
      }
  }
}

// ==============================================================================
// MAIN NODE
// ==============================================================================

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Safe parameter declaration for ROS 2 to avoid override exceptions
  bool debug_mode = true;
  if (!node->has_parameter("debug")) {
    node->declare_parameter<bool>("debug", true);
  }
  debug_mode = node->get_parameter("debug").as_bool();

  // Declare parameter for the text to be traced
  std::string trace_text = "ROMER";
  if (!node->has_parameter("text")) {
    node->declare_parameter<std::string>("text", "ROMER");
  }
  trace_text = node->get_parameter("text").as_string();

  RCLCPP_INFO(logger, "Running in %s mode.", debug_mode ? "DEBUG (RViz Next prompts ON)" : "PRODUCTION (Continuous Execution)");
  RCLCPP_INFO(logger, "Text to trace: %s", trace_text.c_str());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");

  // Wire up the custom Ctrl+C interceptor
  g_mgi = &move_group_interface;
  std::signal(SIGINT, sigint_handler);
  
  move_group_interface.setPlanningTime(5.0); 
  float VEL_SCALE = 0.25; 
  float CART_VEL_SCALE = 0.1; 
  move_group_interface.setMaxVelocityScalingFactor(VEL_SCALE);
  move_group_interface.setMaxAccelerationScalingFactor(VEL_SCALE);

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "ur5e_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // SINGLE REQUIRED START PROMPT
  moveit_visual_tools.prompt("Press 'Next' in RViz to START the Zenpool Sequence!");

  // Adjusted Prompt Lambda based on 'debug' parameter
  auto const prompt = [&moveit_visual_tools, logger, debug_mode](auto text) { 
      if (g_abort) return;
      if (debug_mode) {
          moveit_visual_tools.prompt(text); 
      } else {
          // Convert auto text to string safely for logger
          std::string text_str = text;
          RCLCPP_INFO(logger, "Executing: %s", text_str.c_str());
      }
  };
  
  auto const draw_traj = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_arm")](auto const traj) { 
      if (!g_abort) moveit_visual_tools.publishTrajectoryLine(traj, jmg); 
  };
  
  auto drop_pen_joint_positions = makeJointTarget({ -127.71, -130.59, -67.87, -70.89, 91.66, -173.10 });
  auto get_pen_joint_positions = makeJointTarget({ -124.61, -101.97, -112.05, -55.27, 91.60, -169.91 });
  auto drop_cleaner_joint_positions = makeJointTarget({ -137.01, -132.86, -63.86, -72.83, 91.76, -182.42 });
  auto get_cleaner_joint_positions = makeJointTarget({ -138.01, -104.85, -108.24, -56.47, 91.74, -183.32 });
  auto safe_point_joint_positions = makeJointTarget({-81.36, -88.98, -126.02, -53.74, 90.79, -126.67});

  geometry_msgs::msg::Pose tool_pose;
  tool_pose.orientation.x = 0.0;
  tool_pose.orientation.y = 0.9999846;
  tool_pose.orientation.z = 0.0;
  tool_pose.orientation.w = 0.0055555;
  tool_pose.position.x = 0.0;
  tool_pose.position.y = 0.0;
  tool_pose.position.z = 0.02;


  moveit::planning_interface::PlanningSceneInterface psi;
  Tool pen_tool("pen_tool", "package://my_robot_cell_description/meshes/PenTool.stl", move_group_interface.getEndEffectorLink(), tool_pose);
  Tool cleaner_tool("cleaner_tool", "package://my_robot_cell_description/meshes/CleanerTool.stl", move_group_interface.getEndEffectorLink(), tool_pose);

  // --- 1. CLEANING PHASE ---
  executeJoint(move_group_interface, drop_cleaner_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to drop_cleaner hover");
  executeCartesian(move_group_interface, get_cleaner_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to attach cleaner", CART_VEL_SCALE, CART_VEL_SCALE);
  cleaner_tool.attach(psi);
  executeCartesian(move_group_interface, safe_point_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to safe position", CART_VEL_SCALE, CART_VEL_SCALE);
  
  prompt("Clean sand surface");
  move_group_interface.setEndEffectorLink("ur5e_tool0");
  executeCleanSweep(move_group_interface, logger, draw_traj, moveit_visual_tools, prompt, CART_VEL_SCALE, CART_VEL_SCALE);
  move_group_interface.setEndEffectorLink("ur5e_tool0"); 

  executeCartesian(move_group_interface, safe_point_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to safe position", CART_VEL_SCALE, CART_VEL_SCALE);
  executeJoint(move_group_interface, get_cleaner_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to drop_cleaner hover");
  cleaner_tool.detach(psi);
  executeCartesian(move_group_interface, drop_cleaner_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Retreat from cleaner", CART_VEL_SCALE, CART_VEL_SCALE);

  // --- 2. WRITING PHASE ---
  executeJoint(move_group_interface, drop_pen_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to pen hover");
  executeCartesian(move_group_interface, get_pen_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Attach pen", CART_VEL_SCALE, CART_VEL_SCALE);
  pen_tool.attach(psi);
  executeCartesian(move_group_interface, safe_point_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to safe position", CART_VEL_SCALE, CART_VEL_SCALE);

  prompt("Trace text");
  move_group_interface.setEndEffectorLink("ur5e_tool0");
  // Pass the newly declared parameter trace_text
  executeTextTracing(move_group_interface, trace_text, logger, draw_traj, moveit_visual_tools, prompt, CART_VEL_SCALE, CART_VEL_SCALE);
  move_group_interface.setEndEffectorLink("ur5e_tool0"); 

  executeCartesian(move_group_interface, safe_point_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to safe point", CART_VEL_SCALE, CART_VEL_SCALE);
  executeJoint(move_group_interface, get_pen_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Move to drop pen hover");
  pen_tool.detach(psi);
  executeCartesian(move_group_interface, drop_pen_joint_positions, logger, draw_traj, moveit_visual_tools, prompt, "Retreat from pen", CART_VEL_SCALE, CART_VEL_SCALE);
  
  executeNamed(move_group_interface, "home", logger, draw_traj, moveit_visual_tools, prompt, "Move to home");

  // ==============================================================================
  // AUTOMATIC CLEANUP & EXIT 
  // ==============================================================================
  
  RCLCPP_INFO(logger, "Cleaning up tools and exiting node...");
  
  // Physically delete the objects from the world regardless of what point we aborted
  std::vector<std::string> objects_to_remove = {"pen_tool", "cleaner_tool"};
  psi.removeCollisionObjects(objects_to_remove);

  // Briefly sleep so the ROS network has time to transmit the "Delete Object" command before it shuts down
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Clean disconnect
  g_mgi = nullptr;
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  spinner.join();
  return 0;
}