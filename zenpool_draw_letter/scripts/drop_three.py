#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time

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

class DropPenSequence(Node):
    def __init__(self):
        super().__init__('drop_pen_sequence')
        
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
        self.tool_drop_anchor = [
            -1.9521525541888636, -1.6241785488524378, -1.4479844570159912, 
            -1.6395284138121546, 1.5706689357757568, 3.54525089263916
        ]
        
        self.tool_take = [
            -1.8861225287066858, -2.2894236050047816, -1.1587438583374023, 
            -1.2635646027377625, 1.5706090927124023, 3.6113338470458984
        ]
        
        self.tool_drop = [
            -1.6701200644122522, -1.7745048008360804, -1.946237325668335, 
            -0.9909945887378235, 1.5705393552780151, 3.8273630142211914
        ]
        
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
    
    node = DropPenSequence()
    
    # Wait for controllers to spin up
    time.sleep(1.0)
    
    try:
        # Step 1: Move to Tool Drop Anchor
        node.get_logger().info("=== STEP 1: Moving to Tool Drop Anchor ===")
        if not node.execute_joint_trajectory(node.tool_drop_anchor, sec=4):
            raise Exception("Step 1 Failed!")
            
        # Step 2: Move to Tool Drop
        node.get_logger().info("=== STEP 2: Moving to Tool Drop ===")
        if not node.execute_joint_trajectory(node.tool_drop, sec=3):
            raise Exception("Step 2 Failed!")
            
        # Let controller settle
        time.sleep(0.5)
        
        # Step 3: Detach Tool in Simulation (BEFORE sliding into the tight slot to avoid collision failures)
        node.get_logger().info("=== STEP 3: Detaching Tool in Simulation ===")
        msg = String()
        msg.data = 'detach pen'
        node.tool_cmd_pub.publish(msg)
        
        # VERY IMPORTANT: Wait for tool_manager to update the MoveIt Planning Scene
        # before we ask MoveIt to compute the collision-free Cartesian path!
        time.sleep(1.0)
        
        # Step 4: Cartesian Slide to Tool Take (Robot pushes the physical tool into the slot)
        node.get_logger().info("=== STEP 4: Cartesian Slide to Tool Take ===")
        if not node.execute_cartesian_to_joints(node.tool_take):
            raise Exception("Step 4 Failed!")
            
        node.get_logger().info("SUCCESS! Drop Sequence completed perfectly.")
        
    except Exception as e:
        node.get_logger().error(f"Drop Sequence aborted: {e}")
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
