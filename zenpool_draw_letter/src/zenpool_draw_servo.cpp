#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <cmath>
#include <map>
#include <algorithm>
#include <atomic>
#include <csignal>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "zenpool_draw_letter/font_library.h"

std::atomic<bool> g_abort{false};
moveit::planning_interface::MoveGroupInterface* g_mgi = nullptr;

void sigint_handler(int /*sig*/) {
  g_abort = true;
  if (g_mgi) g_mgi->stop();
}

class Tool {
public:
  Tool(const std::string& id, const std::string& mesh_resource, const std::string& ee_link, const geometry_msgs::msg::Pose& pose)
  : id_(id), ee_link_(ee_link) {
    shapes::Mesh* m = shapes::createMeshFromResource(mesh_resource);
    shapes::ShapeMsg mesh_msg_tmp;
    shapes::constructMsgFromShape(m, mesh_msg_tmp);
    mesh_msg_ = boost::get<shape_msgs::msg::Mesh>(mesh_msg_tmp);
    pose_ = pose;
  }
  void attach(moveit::planning_interface::PlanningSceneInterface& psi) {
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
  void detach(moveit::planning_interface::PlanningSceneInterface& psi, bool remove_from_world = true) {
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.object.id = id_;
    detach_object.link_name = ee_link_;
    detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    psi.applyAttachedCollisionObject(detach_object);
    if (remove_from_world) psi.removeCollisionObjects({ id_ });
  }
private:
  std::string id_, ee_link_;
  shape_msgs::msg::Mesh mesh_msg_;
  geometry_msgs::msg::Pose pose_;
};

std::vector<double> makeJointTarget(const std::vector<double>& deg_values) {
  std::vector<double> rad_values(deg_values.size());
  for (size_t i = 0; i < deg_values.size(); ++i) rad_values[i] = deg_values[i] * M_PI / 180.0;
  return rad_values;
}

void switch_controller(rclcpp::Node::SharedPtr node, const std::string& start, const std::string& stop) {
    auto client = node->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(node->get_logger(), "Waiting for switch_controller service...");
    }
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->start_controllers = {start};
    request->stop_controllers = {stop};
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    auto result = client->async_send_request(request);
    if (result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        RCLCPP_INFO(node->get_logger(), "Switched to %s", start.c_str());
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call switch_controller service");
    }
}

void start_servo(rclcpp::Node::SharedPtr node) {
    auto client = node->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) return;
        RCLCPP_INFO(node->get_logger(), "Waiting for /servo_node/start_servo service...");
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client->async_send_request(request);
    if (result.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
        RCLCPP_INFO(node->get_logger(), "Started MoveIt Servo successfully.");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call start_servo service");
    }
}

bool executeJoint(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& mgi, const std::vector<double>& joint_positions,
                  const rclcpp::Logger& logger, auto& prompt, const std::string& label) {
  if (g_abort) return false;
  prompt("Servo Execute " + label);
  if (g_abort) return false;

  auto jog_pub = node->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);
  rclcpp::Rate rate(100);
  
  std::vector<std::string> joint_names = mgi.getJointNames();

  bool reached = false;
  RCLCPP_INFO(logger, "Starting JointJog loop. Joint positions size: %zu", joint_positions.size());
  
  int loop_count = 0;
  while (rclcpp::ok() && !g_abort && !reached) {
      std::vector<double> current_joints = mgi.getCurrentJointValues();
      
      if (loop_count % 100 == 0) {
          RCLCPP_INFO(logger, "Loop %d: current_joints size: %zu", loop_count, current_joints.size());
      }
      loop_count++;

      if (current_joints.empty() || current_joints.size() != joint_positions.size()) {
          rate.sleep();
          continue;
      }

      control_msgs::msg::JointJog jog_msg;
      jog_msg.header.stamp = node->now();
      jog_msg.header.frame_id = "ur5e_base_link";
      jog_msg.joint_names = joint_names;

      double max_error = 0.0;
      double kp = 3.0; // P-gain
      double max_v = 0.5; // rad/s limit

      for (size_t i = 0; i < joint_positions.size(); ++i) {
          double error = joint_positions[i] - current_joints[i];
          if (std::abs(error) > max_error) max_error = std::abs(error);
          
          double vel = kp * error;
          if (vel > max_v) vel = max_v;
          if (vel < -max_v) vel = -max_v;
          
          jog_msg.velocities.push_back(vel);
      }

      if (loop_count % 100 == 1) {
          RCLCPP_INFO(logger, "Max error: %f. Publishing jog...", max_error);
      }

      if (max_error < 0.02) { 
          reached = true;
          break;
      }

      jog_pub->publish(jog_msg);
      rate.sleep();
  }

  RCLCPP_INFO(logger, "JointJog loop finished!");

  // Stop motion
  control_msgs::msg::JointJog stop_msg;
  stop_msg.header.stamp = node->now();
  stop_msg.header.frame_id = "ur5e_base_link";
  stop_msg.joint_names = joint_names;
  for (size_t i = 0; i < joint_names.size(); ++i) stop_msg.velocities.push_back(0.0);
  jog_pub->publish(stop_msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  return true;
}

bool executeTrajectoryServo(rclcpp::Node::SharedPtr node, const moveit_msgs::msg::RobotTrajectory& traj, const std::vector<std::string>& joint_names, const rclcpp::Logger& logger, auto& prompt, const std::string& label) {
    if (g_abort || traj.joint_trajectory.points.empty()) return false;
    prompt("Servo Executing Trajectory: " + label);
    if (g_abort) return false;

    auto jog_pub = node->create_publisher<control_msgs::msg::JointJog>("/servo_node/delta_joint_cmds", 10);
    rclcpp::Rate rate(100);
    
    auto start_time = node->now();
    double total_duration = rclcpp::Duration(traj.joint_trajectory.points.back().time_from_start).seconds();
    
    while (rclcpp::ok() && !g_abort) {
        double elapsed = (node->now() - start_time).seconds();
        if (elapsed > total_duration) break;

        // Find current target point
        size_t idx = 0;
        for (size_t i = 0; i < traj.joint_trajectory.points.size() - 1; ++i) {
            if (rclcpp::Duration(traj.joint_trajectory.points[i+1].time_from_start).seconds() > elapsed) {
                idx = i;
                break;
            }
        }

        const auto& p1 = traj.joint_trajectory.points[idx];
        const auto& p2 = traj.joint_trajectory.points[idx+1];
        
        double t1 = rclcpp::Duration(p1.time_from_start).seconds();
        double t2 = rclcpp::Duration(p2.time_from_start).seconds();
        double ratio = (elapsed - t1) / (t2 - t1);

        std::vector<double> current_joints = g_mgi->getCurrentJointValues();
        if (current_joints.empty()) { rate.sleep(); continue; }

        control_msgs::msg::JointJog jog_msg;
        jog_msg.header.stamp = node->now();
        jog_msg.header.frame_id = "ur5e_base_link";
        jog_msg.joint_names = joint_names;

        double kp = 5.0; // P-gain for tight tracking
        double max_v = 1.0; 

        for (size_t i = 0; i < p1.positions.size(); ++i) {
            double target_pos = p1.positions[i] + ratio * (p2.positions[i] - p1.positions[i]);
            double error = target_pos - current_joints[i];
            
            // Feed-forward velocity + P-control
            double ff_vel = p1.velocities[i] + ratio * (p2.velocities[i] - p1.velocities[i]);
            double vel = ff_vel + kp * error;
            
            if (vel > max_v) vel = max_v;
            if (vel < -max_v) vel = -max_v;
            jog_msg.velocities.push_back(vel);
        }

        jog_pub->publish(jog_msg);
        rate.sleep();
    }

    control_msgs::msg::JointJog stop_msg;
    stop_msg.header.stamp = node->now();
    stop_msg.header.frame_id = "ur5e_base_link";
    stop_msg.joint_names = joint_names;
    for (size_t i = 0; i < joint_names.size(); ++i) stop_msg.velocities.push_back(0.0);
    jog_pub->publish(stop_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return true;
}

bool executeCartesianServo(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& mgi,
                      const std::vector<double>& target_joint_positions,
                      const rclcpp::Logger& logger, auto& prompt, const std::string& label) {
  if (g_abort) return false;
  moveit::core::RobotStatePtr k_state = std::make_shared<moveit::core::RobotState>(*mgi.getCurrentState());
  k_state->setJointGroupPositions(k_state->getJointModelGroup(mgi.getName()), target_joint_positions);
  const Eigen::Isometry3d& end_state = k_state->getGlobalLinkTransform(mgi.getEndEffectorLink());

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = end_state.translation().x();
  target_pose.position.y = end_state.translation().y();
  target_pose.position.z = end_state.translation().z();
  Eigen::Quaterniond q(end_state.rotation());
  target_pose.orientation.x = q.x(); target_pose.orientation.y = q.y(); target_pose.orientation.z = q.z(); target_pose.orientation.w = q.w();

  std::vector<geometry_msgs::msg::Pose> waypoints = {target_pose};
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mgi.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  if (fraction < 0.9) {
      RCLCPP_WARN(logger, "Cartesian path failed for %s", label.c_str());
      return false;
  }
  return executeTrajectoryServo(node, trajectory, mgi.getJointNames(), logger, prompt, label);
}

void executeCleanSweepServo(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& mgi,
                       const rclcpp::Logger& logger, auto& prompt) 
{
  if (g_abort) return;
  moveit::core::RobotStatePtr current_state = mgi.getCurrentState();
  if (!current_state->knowsFrameTransform("pool")) return;
  const Eigen::Isometry3d& T_base_pool = current_state->getGlobalLinkTransform("pool");
  Eigen::Matrix3d R_cleaner = (Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Quaterniond q_cleaner_base(T_base_pool.rotation() * R_cleaner);

  geometry_msgs::msg::Quaternion cleaner_orient;
  cleaner_orient.x = q_cleaner_base.x(); cleaner_orient.y = q_cleaner_base.y(); cleaner_orient.z = q_cleaner_base.z(); cleaner_orient.w = q_cleaner_base.w();

  double constant_z = -0.115 + 0.115 + 0.02 - 0.019; 
  Eigen::Vector3d sweep_start(-0.5, -0.35, constant_z); 
  Eigen::Vector3d sweep_end(0.5, -0.6, constant_z); 

  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto create_pose = [&](const Eigen::Vector3d& pos) {
      geometry_msgs::msg::Pose p; p.position.x = pos.x(); p.position.y = pos.y(); p.position.z = pos.z(); p.orientation = cleaner_orient; return p;
  };

  waypoints.push_back(create_pose(T_base_pool * (sweep_start + Eigen::Vector3d(0, 0, 0.05))));
  waypoints.push_back(create_pose(T_base_pool * sweep_start));
  waypoints.push_back(create_pose(T_base_pool * Eigen::Vector3d(sweep_end.x(), sweep_start.y(), sweep_start.z())));

  if (g_abort) return;
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = mgi.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  if (fraction > 0.9) {
      executeTrajectoryServo(node, trajectory, mgi.getJointNames(), logger, prompt, "Execute Sweep Area");
  } else {
      RCLCPP_WARN(logger, "Sweep planning failed!");
  }
}

void executeTextTracingServo(rclcpp::Node::SharedPtr node, moveit::planning_interface::MoveGroupInterface& mgi, const std::string& text,
                        const rclcpp::Logger& logger, auto& prompt) 
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
    if (i < chars.size() - 1) unscaled_width += 0.35;
  }
  double scale = 0.8 / unscaled_width; 
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
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = mgi.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
      if (fraction > 0.5) {
          executeTrajectoryServo(node, trajectory, mgi.getJointNames(), logger, prompt, "Tracing " + text);
      } else {
          RCLCPP_WARN(logger, "Text tracing planning failed!");
      }
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("zenpool_draw_servo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("zenpool_draw_servo");

  bool debug_mode = true;
  if (!node->has_parameter("debug")) {
    node->declare_parameter<bool>("debug", true);
  }
  debug_mode = node->get_parameter("debug").as_bool();

  std::string trace_text = "ROMER";
  if (!node->has_parameter("text")) {
    node->declare_parameter<std::string>("text", "ROMER");
  }
  trace_text = node->get_parameter("text").as_string();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_arm");

  g_mgi = &move_group_interface;
  std::signal(SIGINT, sigint_handler);

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "ur5e_base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.prompt("Press 'Next' in RViz to START the Zenpool Sequence!");

  auto const prompt = [&moveit_visual_tools, logger, debug_mode](auto text) { 
      if (g_abort) return;
      if (debug_mode) moveit_visual_tools.prompt(text); 
      else RCLCPP_INFO(logger, "Executing: %s", std::string(text).c_str());
  };
  
  auto const draw_traj = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("ur_arm")](auto const traj) { 
      if (!g_abort) moveit_visual_tools.publishTrajectoryLine(traj, jmg); 
  };
  
  // Ensure the robot is using forward_velocity_controller
  switch_controller(node, "forward_velocity_controller", "scaled_joint_trajectory_controller");
  
  // Wake up MoveIt Servo
  start_servo(node);

  auto drop_pen_joint_positions = makeJointTarget({ -127.71, -130.59, -67.87, -70.89, 91.66, -173.10 });
  auto get_pen_joint_positions = makeJointTarget({ -124.61, -101.97, -112.05, -55.27, 91.60, -169.91 });
  auto drop_cleaner_joint_positions = makeJointTarget({ -137.01, -132.86, -63.86, -72.83, 91.76, -182.42 });
  auto get_cleaner_joint_positions = makeJointTarget({ -138.01, -104.85, -108.24, -56.47, 91.74, -183.32 });
  auto safe_point_joint_positions = makeJointTarget({-81.36, -88.98, -126.02, -53.74, 90.79, -126.67});

  geometry_msgs::msg::Pose tool_pose;
  tool_pose.orientation.y = 0.9999846; tool_pose.orientation.w = 0.0055555; tool_pose.position.z = 0.02;

  moveit::planning_interface::PlanningSceneInterface psi;
  Tool pen_tool("pen_tool", "package://my_robot_cell_description/meshes/PenTool.stl", move_group_interface.getEndEffectorLink(), tool_pose);
  Tool cleaner_tool("cleaner_tool", "package://my_robot_cell_description/meshes/CleanerTool.stl", move_group_interface.getEndEffectorLink(), tool_pose);

  // --- 1. CLEANING PHASE ---
  executeJoint(node, move_group_interface, drop_cleaner_joint_positions, logger, prompt, "Move to drop_cleaner hover");
  executeCartesianServo(node, move_group_interface, get_cleaner_joint_positions, logger, prompt, "Move to attach cleaner");
  cleaner_tool.attach(psi);
  executeCartesianServo(node, move_group_interface, safe_point_joint_positions, logger, prompt, "Move to safe position");
  
  prompt("Clean sand surface");
  move_group_interface.setEndEffectorLink("ur5e_tool0");
  executeCleanSweepServo(node, move_group_interface, logger, prompt);
  move_group_interface.setEndEffectorLink("ur5e_tool0"); 

  executeCartesianServo(node, move_group_interface, safe_point_joint_positions, logger, prompt, "Move to safe position");
  executeJoint(node, move_group_interface, get_cleaner_joint_positions, logger, prompt, "Move to drop_cleaner hover");
  cleaner_tool.detach(psi);
  executeCartesianServo(node, move_group_interface, drop_cleaner_joint_positions, logger, prompt, "Retreat from cleaner");

  // --- 2. WRITING PHASE ---
  executeJoint(node, move_group_interface, drop_pen_joint_positions, logger, prompt, "Move to pen hover");
  executeCartesianServo(node, move_group_interface, get_pen_joint_positions, logger, prompt, "Attach pen");
  pen_tool.attach(psi);
  executeCartesianServo(node, move_group_interface, safe_point_joint_positions, logger, prompt, "Move to safe position");

  prompt("Trace text");
  move_group_interface.setEndEffectorLink("ur5e_tool0");
  executeTextTracingServo(node, move_group_interface, trace_text, logger, prompt);
  move_group_interface.setEndEffectorLink("ur5e_tool0"); 

  executeCartesianServo(node, move_group_interface, safe_point_joint_positions, logger, prompt, "Move to safe point");
  executeJoint(node, move_group_interface, get_pen_joint_positions, logger, prompt, "Move to drop pen hover");
  pen_tool.detach(psi);
  executeCartesianServo(node, move_group_interface, drop_pen_joint_positions, logger, prompt, "Retreat from pen");
  
  // Clean disconnect
  g_mgi = nullptr;
  if (rclcpp::ok()) rclcpp::shutdown();
  spinner.join();
  return 0;
}