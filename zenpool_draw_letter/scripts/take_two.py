#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time
import sys

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def wait_for_future(node, future, timeout_sec=None):
    """Custom spin loop to avoid asyncio deadlock"""
    start = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            return future.result()
        if timeout_sec and (time.time() - start) > timeout_sec:
            node.get_logger().error("Future timed out!")
            return None

class TakeCleanerSequence(Node):
    def __init__(self):
        super().__init__('take_cleaner_sequence')
        
        # Action Client for executing joint movements
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        # Service Clients
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Publisher for Tool Manager
        self.tool_cmd_pub = self.create_publisher(String, '/tool_manager/command', 10)
        
        self.joint_names = [
            'ur5e_shoulder_pan_joint',
            'ur5e_shoulder_lift_joint',
            'ur5e_elbow_joint',
            'ur5e_wrist_1_joint',
            'ur5e_wrist_2_joint',
            'ur5e_wrist_3_joint'
        ]
        
        # Hardcoded anchors from my_robot_cell.srdf
        self.tool_take_anchor = [
            -2.0374296347247522, -1.9833909473814906, -0.9985051155090332, 
            -1.7298089466490687, 1.570746898651123, 3.459979295730591
        ]
        
        self.tool_take = [
            -2.0331299940692347, -2.2137681446471156, -1.2919307947158813, 
            -1.2062910360148926, 1.5706932544708252, 3.464433193206787
        ]
        
        self.tool_drop = [
            -1.8669331709491175, -1.6779171429076136, -2.0657098293304443, 
            -0.9683686655810853, 1.5706557035446167, 3.6306023597717285
        ]
        


    def execute_manhattan_to_joints(self, target_joints):
        self.get_logger().info("Calculating FK for target joint angles to begin Manhattan probe...")
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            return False
            
        req_fk = GetPositionFK.Request()
        req_fk.header.frame_id = "ur5e_base_link"
        req_fk.fk_link_names = ["ur5e_tool0"]
        req_fk.robot_state.joint_state = JointState()
        req_fk.robot_state.joint_state.name = self.joint_names
        req_fk.robot_state.joint_state.position = target_joints
        
        future_fk = self.fk_client.call_async(req_fk)
        result_fk = wait_for_future(self, future_fk)
        if not result_fk or not result_fk.pose_stamped:
            return False
        target_pose = result_fk.pose_stamped[0].pose
        
        req_fk_curr = GetPositionFK.Request()
        req_fk_curr.header.frame_id = "ur5e_base_link"
        req_fk_curr.fk_link_names = ["ur5e_tool0"]
        future_fk_curr = self.fk_client.call_async(req_fk_curr)
        result_fk_curr = wait_for_future(self, future_fk_curr)
        if not result_fk_curr or not result_fk_curr.pose_stamped:
            return False
        curr_pose = result_fk_curr.pose_stamped[0].pose
        
        start_z = curr_pose.position.z
        end_z = target_pose.position.z
        
        req = GetCartesianPath.Request()
        req.header.frame_id = 'ur5e_base_link'
        req.group_name = 'ur_arm'
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        
        speed_factor = 0.5
        time_multiplier = 1.0 / speed_factor
        
        req.waypoints = [target_pose]
        fut_path = self.cartesian_client.call_async(req)
        res_path = wait_for_future(self, fut_path)
        
        if res_path and res_path.fraction >= 0.99:
            self.get_logger().info("Direct path is clear! Executing...")
            traj = res_path.solution.joint_trajectory
            for point in traj.points:
                t_sec = point.time_from_start.sec + (point.time_from_start.nanosec * 1e-9)
                t_sec *= time_multiplier
                point.time_from_start.sec = int(t_sec)
                point.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)
                if point.velocities: point.velocities = [v * speed_factor for v in point.velocities]
                if point.accelerations: point.accelerations = [a * (speed_factor**2) for a in point.accelerations]
            return self.execute_cartesian_trajectory(traj)
            
        self.get_logger().warn(f"Direct path collided. Starting Manhattan Probe...")
        
        clearance_offset = 0.05
        max_clearance = 0.50
        
        while clearance_offset <= max_clearance:
            safe_z = max(start_z, end_z) + clearance_offset
            self.get_logger().info(f"Probing Manhattan path at Z = {safe_z:.3f}m (+{clearance_offset*100:.0f}cm)...")
            
            wp1 = Pose()
            wp1.position.x = curr_pose.position.x
            wp1.position.y = curr_pose.position.y
            wp1.position.z = safe_z
            wp1.orientation = curr_pose.orientation
            
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
            fut_path = self.cartesian_client.call_async(req)
            res_path = wait_for_future(self, fut_path)
            
            if res_path and res_path.fraction >= 0.99:
                self.get_logger().info(f"Safe Manhattan path found at Z = {safe_z:.3f}m! Executing...")
                traj = res_path.solution.joint_trajectory
                for point in traj.points:
                    t_sec = point.time_from_start.sec + (point.time_from_start.nanosec * 1e-9)
                    t_sec *= time_multiplier
                    point.time_from_start.sec = int(t_sec)
                    point.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)
                    if point.velocities: point.velocities = [v * speed_factor for v in point.velocities]
                    if point.accelerations: point.accelerations = [a * (speed_factor**2) for a in point.accelerations]
                return self.execute_cartesian_trajectory(traj)
                
            self.get_logger().warn(f"Collision at Z = {safe_z:.3f}m. Increasing clearance...")
            clearance_offset += 0.05
            
        self.get_logger().error("Manhattan Probe failed! Reached maximum clearance ceiling without finding a safe path.")
        return False

    def execute_joint_trajectory(self, target_joints, sec=3):
        self.get_logger().info(f"Waiting for Trajectory Server...")
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory Server offline")
            return False
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start.sec = sec
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info("Sending joint goal...")
        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        goal_handle = wait_for_future(self, send_goal_future)
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller!")
            return False
            
        self.get_logger().info("Goal accepted. Executing...")
        result_future = goal_handle.get_result_async()
        result = wait_for_future(self, result_future)
        
        if result and result.status == 4: # rclpy.action.GoalStatus.STATUS_SUCCEEDED
            self.get_logger().info("Movement Complete!")
            return True
        else:
            self.get_logger().error("Movement failed or aborted!")
            return False

    def execute_cartesian_trajectory(self, trajectory_msg):
        self.get_logger().info(f"Waiting for Trajectory Server for Cartesian...")
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory Server offline")
            return False
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg
        
        self.get_logger().info("Sending Cartesian goal...")
        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        goal_handle = wait_for_future(self, send_goal_future)
        
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by controller!")
            return False
            
        self.get_logger().info("Cartesian Goal accepted. Executing...")
        result_future = goal_handle.get_result_async()
        result = wait_for_future(self, result_future)
        
        if result and result.status == 4:
            self.get_logger().info("Cartesian Movement Complete!")
            return True
        else:
            self.get_logger().error("Cartesian Movement failed or aborted!")
            return False

    def execute_cartesian_to_joints(self, target_joints):
        """
        Uses FK to find the exact Cartesian pose of target_joints, 
        then asks MoveIt for a straight Cartesian line to that exact pose.
        """
        self.get_logger().info("Calculating Forward Kinematics for target joint angles...")
        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("FK service offline")
            return False
            
        req_fk = GetPositionFK.Request()
        req_fk.header.frame_id = "ur5e_base_link"
        req_fk.fk_link_names = ["ur5e_tool0"]
        req_fk.robot_state.joint_state = JointState()
        req_fk.robot_state.joint_state.name = self.joint_names
        req_fk.robot_state.joint_state.position = target_joints
        
        future_fk = self.fk_client.call_async(req_fk)
        result_fk = wait_for_future(self, future_fk)
        
        if not result_fk or not result_fk.pose_stamped:
            self.get_logger().error("FK calculation failed!")
            return False
            
        target_pose = result_fk.pose_stamped[0].pose
        self.get_logger().info(f"Target Cartesian Pose determined: X={target_pose.position.x:.3f}, Y={target_pose.position.y:.3f}, Z={target_pose.position.z:.3f}")

        # Compute Cartesian Path
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cartesian service offline")
            return False
            
        req = GetCartesianPath.Request()
        req.header.frame_id = 'ur5e_base_link'
        req.group_name = 'ur_arm'
        # Start state is inherently current robot state since we leave it empty
        req.waypoints = [target_pose]
        req.max_step = 0.01  # 1cm resolution
        req.avoid_collisions = True
        
        self.get_logger().info("Asking MoveIt to compute Cartesian path to target pose...")
        future = self.cartesian_client.call_async(req)
        result = wait_for_future(self, future)
        
        if not result:
            return False
            
        if result.fraction < 0.99:
            self.get_logger().error(f"MoveIt could only compute {result.fraction*100:.1f}% of path. Aborting.")
            return False
            
        traj = result.solution.joint_trajectory
        
        # Safe heuristic time parameterization (if MoveIt didn't provide time)
        if len(traj.points) > 1 and traj.points[-1].time_from_start.sec == 0 and traj.points[-1].time_from_start.nanosec == 0:
            cumulative_sec = 0.0
            for point in traj.points:
                cumulative_sec += 0.05 # 50ms per cm
                point.time_from_start.sec = int(cumulative_sec)
                point.time_from_start.nanosec = int((cumulative_sec - int(cumulative_sec)) * 1e9)
                
        # Scale speed safely (Increased to 50% speed for snappier performance!)
        speed_factor = 0.5
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
                
        return self.execute_cartesian_trajectory(traj)


def main(args=None):
    rclpy.init(args=args)
    
    node = TakeCleanerSequence()
    
    # Wait for controllers to spin up
    time.sleep(1.0)
    
    try:
        # Step 1: Move to Tool Take Anchor
        node.get_logger().info("=== STEP 1: Moving to Tool Take Anchor ===")
        if not node.execute_manhattan_to_joints(node.tool_take_anchor):
            raise Exception("Step 1 Failed!")
            
        # Step 2: Move to Tool Take
        node.get_logger().info("=== STEP 2: Moving to Tool Take ===")
        if not node.execute_joint_trajectory(node.tool_take, sec=3):
            raise Exception("Step 2 Failed!")
            
        # Let controller settle
        time.sleep(0.5)
        
        # Step 3: Cartesian Slide to Tool Drop
        node.get_logger().info("=== STEP 3: Cartesian Slide to Tool Drop ===")
        if not node.execute_cartesian_to_joints(node.tool_drop):
            raise Exception("Step 3 Failed!")
            
        # Step 4: Attach Tool in Simulation
        node.get_logger().info("=== STEP 4: Attaching Tool in Simulation ===")
        msg = String()
        msg.data = 'attach cleaner'
        node.tool_cmd_pub.publish(msg)
            
        node.get_logger().info("SUCCESS! Sequence completed perfectly.")
        
    except Exception as e:
        node.get_logger().error(f"Sequence aborted: {e}")
        sys.exit(1)
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
