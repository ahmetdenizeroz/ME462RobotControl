#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time
import argparse
import sys
import json
import subprocess
import os

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float32, Int32, String
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import Pose, Vector3
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = [0.0]*4
    q[0] = sr * cp * cy - cr * sp * sy
    q[1] = cr * sp * cy + sr * cp * sy
    q[2] = cr * cp * sy - sr * sp * cy
    q[3] = cr * cp * cy + sr * sp * sy
    return q

def wait_for_future(node, future, timeout_sec=None):
    start = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            return future.result()
        if timeout_sec and (time.time() - start) > timeout_sec:
            node.get_logger().error("Future timed out!")
            return None

class DrawingJobExecutor(Node):
    def __init__(self, offline_file_path=None):
        super().__init__('drawing_job_executor')
        self.offline_file_path = offline_file_path
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        self.reorient_pub = self.create_publisher(Vector3, '/dashboard/reorient', 10)
        
        self.gripper_pub_l = self.create_publisher(Float32, '/gripper/command_left', 10)
        self.gripper_pub_raw_l = self.create_publisher(Int32, '/gripper/command_raw_left', 10)
        self.gripper_pub_r = self.create_publisher(Float32, '/gripper/command_right', 10)
        self.tool_manager_pub = self.create_publisher(String, '/tool_manager/command', 10)
        
        self.joint_names = [
            'ur5e_shoulder_pan_joint',
            'ur5e_shoulder_lift_joint',
            'ur5e_elbow_joint',
            'ur5e_wrist_1_joint',
            'ur5e_wrist_2_joint',
            'ur5e_wrist_3_joint'
        ]
        
        self.joint_speed = 0.5
        self.cart_speed = 0.4  # Keep Cartesian speed slow for safety on drawing tasks
        
        # --- Physical Canvas Parameters ---
        self.CANVAS_W = 1.00 # 100 cm
        self.CANVAS_H = 0.35 # 35 cm
        self.HOVER_OFFSET = 0.03  # 3 cm relative hover height
        self.DRAW_Z = 0.0 # -1.5 cm (touching canvas)
        self.ERASE_Z = 0.01  # 0 cm (touching canvas)
        self.FINGER_Z = 0.174 # 5 cm (Tutan-Khamun)
        
        # Tool swapping script mappings
        self.TOOL_SEQUENCES = {
            "pen": {"take": "take_three.py", "drop": "drop_three.py"},
            "cleaner": {"take": "take_two.py", "drop": "drop_two.py"},
            "finger": {"take": "take_one.py", "drop": "drop_one.py"}
        }
        
        self.state_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tool_state.txt')
        self.current_tool = self.get_tool_state()
        self.get_logger().info(f"Initialized with tool state: {self.current_tool}")
            
    def sync_tool_state(self):
        if self.current_tool:
            tool_name = "tutankhamun" if self.current_tool == "finger" else self.current_tool
            msg = String()
            msg.data = f"attach {tool_name}"
            self.tool_manager_pub.publish(msg)
            self.get_logger().info(f"Published startup attach command for: {tool_name}")

    def get_tool_state(self):
        if os.path.exists(self.state_file):
            with open(self.state_file, 'r') as f:
                state = f.read().strip()
                if state in ["pen", "cleaner", "finger"]:
                    return state
        return None

    def set_tool_state(self, tool):
        with open(self.state_file, 'w') as f:
            f.write(tool if tool else "none")
        self.current_tool = tool

    def get_tf_transform(self, target_frame, source_frame, timeout_sec=3.0):
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                return self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            except Exception:
                pass
        raise Exception(f"TF Lookup timed out for {target_frame} -> {source_frame}")

    def move_to_safe_center(self):
        try:
            trans = self.get_tf_transform('drawing_area', 'ur5e_tool0')
        except Exception as e:
            self.get_logger().error(f"Cannot move to safe center: {e}")
            return False
            
        target = Pose()
        target.position.x = 0.5
        target.position.y = 0.35
        target.position.z = 0.25
        target.orientation = trans.transform.rotation
        
        self.get_logger().info("Moving to safe Cartesian center (X:50, Y:35, Z:25)...")
        return self.execute_cartesian_path([target])
    def execute_trajectory(self, trajectory_msg):
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory Server offline")
            return False
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg
        
        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        goal_handle = wait_for_future(self, send_goal_future)
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller!")
            return False
            
        result_future = goal_handle.get_result_async()
        result = wait_for_future(self, result_future)
        
        if result and result.status == 4:
            return True
        return False

    def scale_trajectory_speed(self, traj, speed_factor):
        if speed_factor >= 1.0: return traj
        time_multiplier = 1.0 / speed_factor
        for point in traj.points:
            t_sec = point.time_from_start.sec + (point.time_from_start.nanosec * 1e-9)
            t_sec *= time_multiplier
            point.time_from_start.sec = int(t_sec)
            point.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)
            if point.velocities:
                point.velocities = [v * speed_factor for v in point.velocities]
            if point.accelerations:
                point.accelerations = [a * (speed_factor**2) for a in point.accelerations]
        return traj

    def execute_joint_trajectory_safe(self, target_joints):
        self.get_logger().info("Planning safe joint trajectory to ready posture...")
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Motion Plan service offline")
            return False
            
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'ur_arm'
        req.motion_plan_request.num_planning_attempts = 5
        req.motion_plan_request.allowed_planning_time = 5.0
        req.motion_plan_request.max_velocity_scaling_factor = self.joint_speed
        req.motion_plan_request.max_acceleration_scaling_factor = self.joint_speed
        
        c = Constraints()
        for i, name in enumerate(self.joint_names):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = target_joints[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.motion_plan_request.goal_constraints.append(c)
        
        future = self.plan_client.call_async(req)
        result = wait_for_future(self, future)
        
        if not result or result.motion_plan_response.error_code.val != 1:
            self.get_logger().error("Failed to find a safe joint path!")
            return False
            
        traj = result.motion_plan_response.trajectory.joint_trajectory
        traj = self.scale_trajectory_speed(traj, self.joint_speed)
        
        return self.execute_trajectory(traj)

    def execute_cartesian_path(self, waypoints):
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            return False
            
        req = GetCartesianPath.Request()
        req.header.frame_id = 'drawing_area' # Frame attached to origin of canvas
        req.group_name = 'ur_arm'
        req.waypoints = waypoints
        req.max_step = 0.01
        req.avoid_collisions = True
        
        future = self.cartesian_client.call_async(req)
        result = wait_for_future(self, future)
        
        if not result or result.fraction < 0.99:
            self.get_logger().error(f"Failed Cartesian Path! (Fraction: {result.fraction if result else 'None'})")
            return False
            
        traj = result.solution.joint_trajectory
        if len(traj.points) > 1 and traj.points[-1].time_from_start.sec == 0 and traj.points[-1].time_from_start.nanosec == 0:
            cumulative_sec = 0.0
            for point in traj.points:
                cumulative_sec += 0.05
                point.time_from_start.sec = int(cumulative_sec)
                point.time_from_start.nanosec = int((cumulative_sec - int(cumulative_sec)) * 1e9)
                
        traj = self.scale_trajectory_speed(traj, self.cart_speed)
        return self.execute_trajectory(traj)

    def run_offline_job(self):
        if not self.offline_file_path or not os.path.exists(self.offline_file_path):
            self.get_logger().error(f"Offline job file {self.offline_file_path} not found!")
            return

        with open(self.offline_file_path, 'r') as f:
            job = json.load(f)
        self.execute_job_dict(job)

    def start_tcp_server(self):
        import socket
        HOST = '0.0.0.0'
        PORT = 9090
        
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((HOST, PORT))
        server_socket.listen(1)
        self.get_logger().info(f"TCP Server listening on {HOST}:{PORT}")
        
        while rclpy.ok():
            try:
                server_socket.settimeout(1.0)
                conn, addr = server_socket.accept()
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"Socket error: {e}")
                break
                
            with conn:
                self.get_logger().info(f"Connected by {addr}")
                data = b""
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                    if b"\n" in chunk:
                        break
                
                if data:
                    try:
                        job = json.loads(data.decode('utf-8').strip())
                        self.execute_job_dict(job)
                        conn.sendall(b"SUCCESS\n")
                    except Exception as e:
                        self.get_logger().error(f"Failed to execute TCP job: {e}")
                        conn.sendall(b"FAILED\n")

    def execute_job_dict(self, job):
        self.get_logger().info(f"Loaded job {job.get('job_id')}")
        
        self.get_logger().info("Enforcing perfectly square wrist orientation before starting job...")
        reorient_msg = Vector3()
        reorient_msg.x = 0.0
        reorient_msg.y = 0.0
        reorient_msg.z = 0.0
        self.reorient_pub.publish(reorient_msg)
        time.sleep(3.5)
        
        has_started_strokes = False
        
        for action in job.get("actions", []):
            atype = action.get("type")
            
            if atype == "same":
                continue
                
            if atype == "erase_all":
                self.get_logger().info("Erase All detected! Executing full_cleaning.py...")
                if self.current_tool != "cleaner":
                    if self.current_tool:
                        self.get_logger().info(f"Moving to safe posture before dropping {self.current_tool}...")
                        self.move_to_safe_center()
                        self.get_logger().info(f"Dropping {self.current_tool} before erase_all...")
                        subprocess.run(["ros2", "run", "zenpool_draw_letter", self.TOOL_SEQUENCES[self.current_tool]["drop"]], check=True)
                        self.set_tool_state(None)
                    self.get_logger().info("Taking cleaner for erase_all...")
                    subprocess.run(["ros2", "run", "zenpool_draw_letter", self.TOOL_SEQUENCES["cleaner"]["take"]], check=True)
                    self.set_tool_state("cleaner")
                    
                subprocess.run(["ros2", "run", "zenpool_draw_letter", "full_cleaning.py"], check=True)
                continue
                
            if atype == "draw":
                desired_tool = "pen"
            elif atype == "erase_squeegee":
                desired_tool = "cleaner"
            elif atype == "erase_finger":
                desired_tool = "finger"
            else:
                self.get_logger().warn(f"Unknown action type: {atype}")
                continue
            if self.current_tool != desired_tool:
                if self.current_tool:
                    self.get_logger().info(f"Moving to safe posture before dropping {self.current_tool}...")
                    self.move_to_safe_center()
                    self.get_logger().info(f"Swapping {self.current_tool} for {desired_tool}")
                    subprocess.run(["ros2", "run", "zenpool_draw_letter", self.TOOL_SEQUENCES[self.current_tool]["drop"]], check=True)
                    self.set_tool_state(None)
                else:
                    self.get_logger().info(f"Taking {desired_tool}")
                
                if desired_tool == "finger":
                    self.get_logger().info("Configuring Tutan-Khamun fingers for erasing (L:1200, R:Open)...")
                    msg_l = Int32()
                    msg_l.data = 1200
                    self.gripper_pub_raw_l.publish(msg_l)
                    
                    msg_r = Float32()
                    msg_r.data = 0.0
                    self.gripper_pub_r.publish(msg_r)
                    
                    time.sleep(1.0)
                
                subprocess.run(["ros2", "run", "zenpool_draw_letter", self.TOOL_SEQUENCES[desired_tool]["take"]], check=True)
                self.set_tool_state(desired_tool)
                

                
            points = action.get("points", [])
            if not points: 
                continue
                
            # --- Spatial Downsampling ---
            min_dist_m = 0.002 if atype == "erase_finger" else 0.005
            if len(points) > 2:
                downsampled = [points[0]]
                last_pt = points[0]
                for pt in points[1:-1]:
                    dx = (pt[0] - last_pt[0]) * self.CANVAS_W
                    dy = (pt[1] - last_pt[1]) * self.CANVAS_H
                    if math.hypot(dx, dy) >= min_dist_m:
                        downsampled.append(pt)
                        last_pt = pt
                downsampled.append(points[-1])
                self.get_logger().info(f"Downsampled stroke from {len(points)} to {len(downsampled)} points to protect DDS buffers.")
                points = downsampled
            # ----------------------------
                
            if not has_started_strokes:
                self.get_logger().info("Moving to safe center before starting strokes...")
                self.move_to_safe_center()
                has_started_strokes = True
                
            # Get perfectly square base orientation first
            try:
                trans = self.get_tf_transform('drawing_area', 'ur5e_tool0')
            except Exception as e:
                self.get_logger().error(f"Cannot get orientation: {e}")
                continue
                
            base_orientation = trans.transform.rotation
            
            # Apply dynamic rotation if this is a Squeegee or Finger stroke
            if atype in ["erase_squeegee", "erase_finger"] and "yaw" in action:
                target_yaw = action.get("yaw", 0.0)
                new_q = quaternion_from_euler(math.pi, 0.0, target_yaw)
                orientation = Pose().orientation
                orientation.x = new_q[0]
                orientation.y = new_q[1]
                orientation.z = new_q[2]
                orientation.w = new_q[3]
                self.get_logger().info(f"Rotating tool to trajectory angle: {math.degrees(target_yaw):.1f} deg")
            else:
                orientation = base_orientation
                
            if atype == "draw":
                working_z = self.DRAW_Z
            elif atype == "erase_finger":
                working_z = self.FINGER_Z
            else:
                working_z = self.ERASE_Z
            
            offset_x, offset_y = 0.0, 0.0
            self.get_logger().info(f"Executing {atype} stroke with {len(points)} points...")
            
            # Hover to first point
            first_pt = points[0]
            hover_pose = Pose()
            hover_pose.position.x = first_pt[0] * self.CANVAS_W + offset_x
            hover_pose.position.y = first_pt[1] * self.CANVAS_H + offset_y
            hover_pose.position.z = working_z + self.HOVER_OFFSET
            hover_pose.orientation = orientation
            if not self.execute_cartesian_path([hover_pose]):
                raise Exception("Failed hover approach")
            
            # Plunge
            plunge_pose = Pose()
            plunge_pose.position.x = hover_pose.position.x
            plunge_pose.position.y = hover_pose.position.y
            plunge_pose.position.z = working_z
            plunge_pose.orientation = orientation
            if not self.execute_cartesian_path([plunge_pose]):
                raise Exception("Failed tool plunge")
            
            # Trace remaining points
            if len(points) > 1:
                waypoints = []
                for pt in points[1:]:
                    p = Pose()
                    p.position.x = pt[0] * self.CANVAS_W + offset_x
                    p.position.y = pt[1] * self.CANVAS_H + offset_y
                    p.position.z = working_z
                    p.orientation = orientation
                    waypoints.append(p)
                    
                if not self.execute_cartesian_path(waypoints):
                    raise Exception("Failed drawing path")
                
            # Lift back to hover
            lift_pose = Pose()
            lift_pose.position.x = waypoints[-1].position.x if len(points) > 1 else plunge_pose.position.x
            lift_pose.position.y = waypoints[-1].position.y if len(points) > 1 else plunge_pose.position.y
            lift_pose.position.z = working_z + self.HOVER_OFFSET
            lift_pose.orientation = orientation
            if not self.execute_cartesian_path([lift_pose]):
                raise Exception("Failed tool lift")
                
        self.get_logger().info("Job fully executed! Returning to safe Cartesian center...")
        self.move_to_safe_center()

def main(args=None):
    parser = argparse.ArgumentParser(description='Drawing Job Executor (TCP Server & Offline)')
    parser.add_argument('--offline', type=str, default=None, help='Absolute path to job JSON file for offline execution')
    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:])
    
    rclpy.init(args=ros_args)
    node = DrawingJobExecutor(parsed_args.offline)
    
    # Wait for tf tree to populate and publishers to establish
    time.sleep(2.0)
    
    # Sync the tool state with MoveIt now that the publisher is ready
    node.sync_tool_state()
    
    try:
        if parsed_args.offline:
            node.get_logger().info("Running in OFFLINE mode...")
            node.run_offline_job()
        else:
            node.get_logger().info("Running in TCP SERVER mode...")
            node.start_tcp_server()
    except Exception as e:
        node.get_logger().error(f"Fatal executor error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
