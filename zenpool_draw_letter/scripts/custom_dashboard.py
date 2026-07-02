#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
import math
import time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from geometry_msgs.msg import Pose, Vector3
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# --- Math Helpers ---
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

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

class HeadlessDashboardNode(Node):
    def __init__(self):
        super().__init__('custom_dashboard')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cb_group = ReentrantCallbackGroup()
        
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=self.cb_group)
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        self.joint_names = [
            'ur5e_shoulder_pan_joint',
            'ur5e_shoulder_lift_joint',
            'ur5e_elbow_joint',
            'ur5e_wrist_1_joint',
            'ur5e_wrist_2_joint',
            'ur5e_wrist_3_joint'
        ]
        
        self.speed_factor = 0.2  # Default to 20% speed for safety!

        # Subscribers for Terminal Commands
        self.sub_speed = self.create_subscription(Float32MultiArray, '/dashboard/set_speed', self.cmd_speed_callback, 10)
        self.sub_joints = self.create_subscription(Float32MultiArray, '/dashboard/move_joints', self.cmd_joints_callback, 10)
        self.sub_cartesian = self.create_subscription(Vector3, '/dashboard/move_cartesian', self.cmd_cartesian_callback, 10)
        self.sub_abs_cartesian = self.create_subscription(Vector3, '/dashboard/move_absolute_cartesian', self.cmd_abs_cartesian_callback, 10)
        self.sub_manhattan_cartesian = self.create_subscription(Vector3, '/dashboard/move_manhattan_cartesian', self.cmd_manhattan_cartesian_callback, 10, callback_group=self.cb_group)
        self.sub_abs_manhattan_cartesian = self.create_subscription(Vector3, '/dashboard/move_absolute_manhattan', self.cmd_abs_manhattan_cartesian_callback, 10, callback_group=self.cb_group)
        self.sub_orient = self.create_subscription(Vector3, '/dashboard/reorient', self.cmd_orient_callback, 10)
        
        self.current_joints = None
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

        self.get_logger().info("Headless Dashboard Ready! Listening to /dashboard topics...")
        
    def js_cb(self, msg: JointState):
        joints = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joints.append(msg.position[idx])
            else:
                return
        self.current_joints = joints

    def cmd_speed_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 1:
            self.get_logger().error("Send exactly 1 value (0.0 to 1.0) for speed factor!")
            return
        
        val = msg.data[0]
        if val <= 0.0 or val > 1.0:
            self.get_logger().error("Speed factor must be between 0.01 and 1.0!")
            return
            
        self.speed_factor = val
        self.get_logger().info(f"Velocity Scaling set to {self.speed_factor*100:.1f}%")

    def scale_trajectory_speed(self, traj):
        if self.speed_factor >= 1.0:
            return traj
            
        time_multiplier = 1.0 / self.speed_factor
        
        for point in traj.points:
            t_sec = point.time_from_start.sec + (point.time_from_start.nanosec * 1e-9)
            t_sec *= time_multiplier
            
            point.time_from_start.sec = int(t_sec)
            point.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)
            
            if point.velocities:
                point.velocities = [v * self.speed_factor for v in point.velocities]
            if point.accelerations:
                point.accelerations = [a * (self.speed_factor**2) for a in point.accelerations]
                
        return traj

    def cmd_joints_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 6:
            self.get_logger().error("You must provide exactly 6 joint angles (in degrees)!")
            return
            
        rads = [math.radians(v) for v in msg.data]
        self.get_logger().info(f"Commanding Joints: {msg.data} degrees")
        self.send_joints(rads)

    def cmd_cartesian_callback(self, msg: Vector3):
        curr = self.get_current_pose()
        if not curr:
            return
            
        self.get_logger().info(f"Commanding Relative Cartesian: X:{msg.x}cm Y:{msg.y}cm Z:{msg.z}cm (Table Frame)")
        dx_table = msg.x / 100.0
        dy_table = msg.y / 100.0
        dz_table = msg.z / 100.0
        
        # The robot base is mounted +45 degrees relative to the table.
        # We rotate the table vector into the robot's base frame.
        # Note: If the robot moves in the wrong diagonal, change this to -45.0!
        theta = math.radians(45.0)
        dx_base = dx_table * math.cos(theta) - dy_table * math.sin(theta)
        dy_base = dx_table * math.sin(theta) + dy_table * math.cos(theta)
        dz_base = dz_table
        
        target = Pose()
        target.position.x = curr.transform.translation.x + dx_base
        target.position.y = curr.transform.translation.y + dy_base
        target.position.z = curr.transform.translation.z + dz_base
        target.orientation = curr.transform.rotation
        
        self.calculate_cartesian_path(target)

    def cmd_abs_cartesian_callback(self, msg: Vector3):
        curr = self.get_current_pose()
        if not curr:
            return
            
        self.get_logger().info(f"Commanding Absolute Cartesian: X:{msg.x}cm Y:{msg.y}cm Z:{msg.z}cm (Table Frame)")
        x_table = msg.x / 100.0
        y_table = msg.y / 100.0
        z_table = msg.z / 100.0
        
        # Rotate table absolute coordinates into robot's base frame
        theta = math.radians(45.0)
        x_base = x_table * math.cos(theta) - y_table * math.sin(theta)
        y_base = x_table * math.sin(theta) + y_table * math.cos(theta)
        z_base = z_table
        
        target = Pose()
        target.position.x = x_base
        target.position.y = y_base
        target.position.z = z_base
        target.orientation = curr.transform.rotation
        
        self.calculate_cartesian_path(target)

    def cmd_manhattan_cartesian_callback(self, msg: Vector3):
        curr = self.get_current_pose()
        if not curr: return
        
        self.get_logger().info(f"Commanding Relative Manhattan: X:{msg.x}cm Y:{msg.y}cm Z:{msg.z}cm")
        dx_table, dy_table, dz_table = msg.x / 100.0, msg.y / 100.0, msg.z / 100.0
        
        theta = math.radians(45.0)
        dx_base = dx_table * math.cos(theta) - dy_table * math.sin(theta)
        dy_base = dx_table * math.sin(theta) + dy_table * math.cos(theta)
        
        target = Pose()
        target.position.x = curr.transform.translation.x + dx_base
        target.position.y = curr.transform.translation.y + dy_base
        target.position.z = curr.transform.translation.z + dz_table
        target.orientation = curr.transform.rotation
        
        self.calculate_manhattan_path(target)

    def cmd_abs_manhattan_cartesian_callback(self, msg: Vector3):
        curr = self.get_current_pose()
        if not curr: return
        
        self.get_logger().info(f"Commanding Absolute Manhattan: X:{msg.x}cm Y:{msg.y}cm Z:{msg.z}cm")
        x_table, y_table, z_table = msg.x / 100.0, msg.y / 100.0, msg.z / 100.0
        
        theta = math.radians(45.0)
        x_base = x_table * math.cos(theta) - y_table * math.sin(theta)
        y_base = x_table * math.sin(theta) + y_table * math.cos(theta)
        
        target = Pose()
        target.position.x = x_base
        target.position.y = y_base
        target.position.z = z_table
        target.orientation = curr.transform.rotation
        
        self.calculate_manhattan_path(target)

    def cmd_orient_callback(self, msg: Vector3):
        curr = self.get_current_pose()
        if not curr:
            return
            
        self.get_logger().info(f"Commanding Base-Parallel Orientation: Pitch(X):{msg.x}deg Roll(Y):{msg.y}deg Yaw(Z):{msg.z}deg (Table Frame)")
        
        # Roll affects X angle, Pitch affects Y angle, Yaw affects Z angle
        # Base parallel baseline = 180 deg roll (pi)
        # We also offset the yaw by 45 degrees to perfectly align Z=0 with the physical table frame!
        table_offset_deg = 45.0
        
        target_roll = math.pi + math.radians(msg.x)
        target_pitch = 0.0 + math.radians(msg.y)
        target_yaw = math.radians(msg.z + table_offset_deg)
        
        new_q = quaternion_from_euler(target_roll, target_pitch, target_yaw)
        
        target = Pose()
        target.position.x = curr.transform.translation.x
        target.position.y = curr.transform.translation.y
        target.position.z = curr.transform.translation.z
        target.orientation.x = new_q[0]
        target.orientation.y = new_q[1]
        target.orientation.z = new_q[2]
        target.orientation.w = new_q[3]
        
        joints = self.calculate_ik(target)
        if joints:
            self.send_joints(joints)

    def send_joints(self, joint_angles_rad):
        if not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Trajectory Server offline")
            return
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        duration = 3.0
        if self.current_joints:
            max_dist = max(abs(t - c) for t, c in zip(joint_angles_rad, self.current_joints))
            speed = max(0.05, 0.5 * self.speed_factor)
            calc_time = max_dist / speed
            duration = max(1.0, calc_time)

        point = JointTrajectoryPoint()
        point.positions = joint_angles_rad
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        self.traj_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Trajectory Sent to MoveIt successfully! (Duration: {duration:.1f}s)")

    def send_trajectory(self, trajectory_msg: JointTrajectory):
        if not self.traj_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Trajectory Server offline")
            return
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg
        
        self.traj_client.send_goal_async(goal_msg)
        self.get_logger().info("Cartesian Straight-Line Trajectory Sent to MoveIt successfully!")

    def calculate_cartesian_path(self, target_pose: Pose):
        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Cartesian Path service offline")
            return
            
        req = GetCartesianPath.Request()
        req.header.frame_id = 'ur5e_base_link'
        req.group_name = 'ur_arm'
        req.waypoints = [target_pose]
        req.max_step = 0.01  # 1 cm resolution
        req.jump_threshold = 0.0 # disable jump threshold
        req.avoid_collisions = True
        
        try:
            future = self.cartesian_client.call_async(req)
            future.add_done_callback(self.cartesian_response_callback)
        except Exception as e:
            self.get_logger().error(f"Cartesian Call Error: {e}")

    def process_and_send_cartesian_result(self, result):
        traj = result.solution.joint_trajectory
        
        # Heuristic Time Parameterization Fallback
        if len(traj.points) > 1 and traj.points[-1].time_from_start.sec == 0 and traj.points[-1].time_from_start.nanosec == 0:
            self.get_logger().info("Applying heuristic time parameterization to Cartesian path...")
            cumulative_sec = 0.0
            for point in traj.points:
                cumulative_sec += 0.05  # 50ms per 1cm waypoint
                point.time_from_start.sec = int(cumulative_sec)
                point.time_from_start.nanosec = int((cumulative_sec - int(cumulative_sec)) * 1e9)
        
        # Apply user-defined velocity scaling
        traj = self.scale_trajectory_speed(traj)
        
        self.send_trajectory(traj)

    def cartesian_response_callback(self, future):
        try:
            result = future.result()
            if result.fraction < 0.99:
                self.get_logger().warn(f"Warning: MoveIt could only compute {result.fraction*100:.1f}% of the straight line. Singularity or Collision detected! Aborting.")
                return
            self.process_and_send_cartesian_result(result)
        except Exception as e:
            self.get_logger().error(f"Cartesian Future Error: {e}")

    def calculate_manhattan_path(self, target_pose: Pose):
        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Cartesian Path service offline")
            return
            
        curr = self.get_current_pose()
        if not curr:
            return
            
        start_z = curr.transform.translation.z
        end_z = target_pose.position.z
        
        req = GetCartesianPath.Request()
        req.header.frame_id = 'ur5e_base_link'
        req.group_name = 'ur_arm'
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        # Step 1: Try direct line
        req.waypoints = [target_pose]
        self.get_logger().info("Trying direct Cartesian path...")
        result = self.cartesian_client.call(req)
        
        if result.fraction >= 0.99:
            self.get_logger().info("Direct path is clear! Executing...")
            self.process_and_send_cartesian_result(result)
            return
            
        self.get_logger().warn(f"Direct path collided (fraction {result.fraction}). Starting Manhattan Probe...")
        
        clearance_offset = 0.05 # 5cm
        max_clearance = 0.50 # 50cm
        
        while clearance_offset <= max_clearance:
            safe_z = max(start_z, end_z) + clearance_offset
            self.get_logger().info(f"Probing Manhattan path at Z = {safe_z:.3f}m (+{clearance_offset*100:.0f}cm)...")
            
            wp1 = Pose()
            wp1.position.x = curr.transform.translation.x
            wp1.position.y = curr.transform.translation.y
            wp1.position.z = safe_z
            wp1.orientation = curr.transform.rotation
            
            wp2 = Pose()
            wp2.position.x = target_pose.position.x
            wp2.position.y = target_pose.position.y
            wp2.position.z = safe_z
            wp2.orientation = target_pose.orientation
            
            wp3 = Pose()
            wp3.position.x = target_pose.position.x
            wp3.position.y = target_pose.position.y
            wp3.position.z = end_z
            wp3.orientation = target_pose.orientation
            
            req.waypoints = [wp1, wp2, wp3]
            result = self.cartesian_client.call(req)
            
            if result.fraction >= 0.99:
                self.get_logger().info(f"Safe Manhattan path found at Z = {safe_z:.3f}m! Executing...")
                self.process_and_send_cartesian_result(result)
                return
                
            self.get_logger().warn(f"Collision at Z = {safe_z:.3f}m. Increasing clearance...")
            clearance_offset += 0.05
            
        self.get_logger().error("Manhattan Probe failed! Reached maximum clearance ceiling (50cm) without finding a safe path.")

    def calculate_ik(self, target_pose: Pose):
        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("IK service offline")
            return None
            
        req = GetPositionIK.Request()
        req.ik_request.group_name = 'ur_arm'
        req.ik_request.pose_stamped.header.frame_id = 'ur5e_base_link'
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True
        
        try:
            future = self.ik_client.call_async(req)
            # Since rclpy.spin() is running, we MUST yield back to the event loop.
            # However, this is a synchronous callback, we cannot yield here safely without blocking the thread!
            # Let's wait asynchronously or just block (bad practice but works in simple cases).
            # Wait! Since we are in a single threaded executor, blocking will deadlock if we wait on a future!
            # The safe way in a callback is to add a done_callback.
            future.add_done_callback(self.ik_response_callback)
            return None
        except Exception as e:
            self.get_logger().error(f"IK Call Error: {e}")
            return None

    def ik_response_callback(self, future):
        try:
            result = future.result()
            if result.error_code.val == 1:
                joints = []
                for name in self.joint_names:
                    idx = result.solution.joint_state.name.index(name)
                    joints.append(result.solution.joint_state.position[idx])
                self.send_joints(joints)
            else:
                self.get_logger().error(f"IK Failed! MoveIt error code: {result.error_code.val}")
        except Exception as e:
            self.get_logger().error(f"IK Future Error: {e}")

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('ur5e_base_link', 'ur5e_tool0', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            return trans
        except Exception as e:
            self.get_logger().error(f"TF Lookup Failed! Cannot get tool0 location: {e}")
            return None

def main():
    rclpy.init()
    node = HeadlessDashboardNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
