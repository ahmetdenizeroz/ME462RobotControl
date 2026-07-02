#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import subprocess
import sys

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK
from moveit_msgs.msg import Constraints, JointConstraint, PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool
from control_msgs.action import GripperCommand

def wait_for_future(node, future, timeout_sec=None):
    start = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            return future.result()
        if timeout_sec and (time.time() - start) > timeout_sec:
            node.get_logger().error("Future timed out!")
            return None

class ActiveGripperingSequence(Node):
    def __init__(self):
        super().__init__('active_grippering_sequence')
        
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper/gripper_action')
        
        self.joint_names = [
            'ur5e_shoulder_pan_joint',
            'ur5e_shoulder_lift_joint',
            'ur5e_elbow_joint',
            'ur5e_wrist_1_joint',
            'ur5e_wrist_2_joint',
            'ur5e_wrist_3_joint'
        ]

    def abort(self, reason):
        self.get_logger().error(f"CRITICAL FAILURE: {reason}. Aborting sequence immediately!")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    def execute_gripper_action(self, position, max_effort):
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.abort("Gripper Action Server offline")
            
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(max_effort)
        
        send_goal_future = self.gripper_client.send_goal_async(goal_msg)
        goal_handle = wait_for_future(self, send_goal_future)
        
        if not goal_handle or not goal_handle.accepted:
            self.abort("Gripper goal rejected")
            
        result_future = goal_handle.get_result_async()
        result = wait_for_future(self, result_future)
        
        if not result or not result.result.reached_goal:
            self.abort(f"Gripper action failed or timed out!")
        return True

    def scale_trajectory_speed(self, traj, speed_factor):
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

    def execute_trajectory(self, trajectory_msg):
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.abort("Trajectory Server offline")
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg
        send_goal_future = self.traj_client.send_goal_async(goal_msg)
        goal_handle = wait_for_future(self, send_goal_future)
        if not goal_handle or not goal_handle.accepted:
            self.abort("Trajectory goal rejected by controller")
        result_future = goal_handle.get_result_async()
        result = wait_for_future(self, result_future)
        if not result or result.status != 4:
            self.abort(f"Trajectory execution failed with status: {result.status if result else 'None'}")
        return True

    def execute_joint_move(self, target_joints):
        self.get_logger().info(f"Moving to joints: {target_joints}")
        req = GetMotionPlan.Request()
        req.motion_plan_request.workspace_parameters.header.frame_id = 'ur5e_base_link'
        req.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        req.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        req.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        req.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        req.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        req.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        req.motion_plan_request.group_name = 'ur_arm'
        req.motion_plan_request.num_planning_attempts = 50
        req.motion_plan_request.allowed_planning_time = 10.0
        
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
            self.abort(f"Failed to find a safe joint path! Error Code: {result.motion_plan_response.error_code.val if result else 'None'}")
            
        traj = result.motion_plan_response.trajectory.joint_trajectory
        traj = self.scale_trajectory_speed(traj, 0.3) # 50% speed
        return self.execute_trajectory(traj)

    def get_current_fk(self, frame_id='pool_actual'):
        req_fk = GetPositionFK.Request()
        req_fk.header.frame_id = frame_id
        req_fk.fk_link_names = ["tc_arm_side"]
        req_fk.robot_state.joint_state = JointState() # use current
        future_fk = self.fk_client.call_async(req_fk)
        result_fk = wait_for_future(self, future_fk)
        if not result_fk or not result_fk.pose_stamped:
            return None
        return result_fk.pose_stamped[0].pose

    def execute_cartesian_path(self, waypoints, frame_id='pool_actual'):
        req = GetCartesianPath.Request()
        req.header.frame_id = frame_id
        req.group_name = 'ur_arm'
        req.waypoints = waypoints
        req.max_step = 0.02
        req.avoid_collisions = False # Deactivated!
        
        future = self.cartesian_client.call_async(req)
        result = wait_for_future(self, future)
        if not result or result.fraction < 0.99:
            self.abort(f"Cartesian path failed. Fraction: {result.fraction if result else 'None'}")
            
        traj = result.solution.joint_trajectory
        traj = self.scale_trajectory_speed(traj, 0.3) # 50% speed
        return self.execute_trajectory(traj)

    def execute_absolute_cartesian(self, x, y, z, frame_id='ur5e_base_link'):
        self.get_logger().info(f"Moving absolute cartesian: [{x}, {y}, {z}] in {frame_id}")
        curr = self.get_current_fk(frame_id)
        if not curr: self.abort("Failed to get current Forward Kinematics!")
        target = Pose()
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.orientation = curr.orientation
        return self.execute_cartesian_path([target], frame_id)

    def execute_relative_cartesian(self, dx, dy, dz, frame_id='pool_actual'):
        self.get_logger().info(f"Moving relative cartesian: dx={dx}, dy={dy}, dz={dz} in {frame_id}")
        curr = self.get_current_fk(frame_id)
        if not curr: self.abort("Failed to get current Forward Kinematics!")
        target = Pose()
        target.position.x = curr.position.x + dx
        target.position.y = curr.position.y + dy
        target.position.z = curr.position.z + dz
        target.orientation = curr.orientation
        return self.execute_cartesian_path([target], frame_id)

def main(args=None):
    rclpy.init(args=args)
    node = ActiveGripperingSequence()
    time.sleep(1.0)
    
    node.get_logger().info("1. Running take_three.py sequence")
    subprocess.run(["ros2", "run", "zenpool_draw_letter", "take_three.py"])
    
    node.get_logger().info("--> DROPPING PEN IN SIMULATION TO IGNORE COLLISIONS <--")
    subprocess.run(["ros2", "topic", "pub", "--once", "/tool_manager/command", "std_msgs/msg/String", "{data: 'detach pen'}"])
    time.sleep(1.0)
    
    node.get_logger().info("2. Moving to joint pos 1")
    node.execute_joint_move([-1.405163590108053, -1.438101978307106, -2.298321008682251, -0.9752960962108155, 1.5702581405639648, 4.092392921447754])
    
    node.get_logger().info("3. Moving to joint pos 2")
    node.execute_joint_move([-1.4051998297320765, -1.4380645018867035, -2.2984459400177, -0.9753621381572266, -1.5702581405639648, 1.5663607120513916])
    
    node.get_logger().info("4. Moving cartesian to [0.070, 0.385, 0.161]")
    node.execute_absolute_cartesian(0.070, 0.385, 0.161)
    
    node.get_logger().info("5. Moving to joint pos 3")
    node.execute_joint_move([-2.376023594533102, -2.4670406780638636, -2.4084599018096924, 0.16364042341198726, -1.5664861837970179, 1.5624985694885254])
    
    node.get_logger().info("6. Moving cartesian to [0.343, 0.144, 0.092]")
    node.execute_absolute_cartesian(0.343, 0.144, 0.092)
    
    node.get_logger().info("7. Moving to joint pos 4")
    node.execute_joint_move([-2.3761847654925745, -2.7163874111571253, -2.3565967082977295, -1.2143813234618683, -1.5532806555377405, 1.5578625202178955])
    
    node.get_logger().info("8. Sending OPEN to gripper")
    node.execute_gripper_action(position=0.0, max_effort=0.0)
    
    node.get_logger().info("9. Moving to joint pos 5")
    node.execute_joint_move([-2.389397923146383, -2.420762678185934, -1.8694475889205933, -1.993124624291891, -1.6041844526873987, 1.5708009004592896])
    
    node.get_logger().info("10. Sending GRASP to gripper")
    node.execute_gripper_action(position=1.0, max_effort=1.0)
    
    node.get_logger().info("Operation paused. Waiting for user input...")
    input("Press Enter to proceed further...")
    
    node.get_logger().info("11. Moving 20cm in -Y")
    node.execute_relative_cartesian(0.0, -0.2, 0.0)
    
    node.get_logger().info("12. Moving 20cm in -X")
    node.execute_relative_cartesian(-0.2, 0.0, 0.0)
    
    node.get_logger().info("13. Moving 80cm in +Y")
    node.execute_relative_cartesian(0.0, 0.8, 0.0)
    
    node.get_logger().info("14. Moving 30cm in +Z")
    node.execute_relative_cartesian(0.0, 0.0, 0.3)
    
    node.get_logger().info("15. Running take_one.py sequence")
    subprocess.run(["ros2", "run", "zenpool_draw_letter", "take_one.py"])
    
    node.get_logger().info("16. Moving to absolute cartesian x: 0 y: 40 z: 30")
    node.execute_absolute_cartesian(0.0, 0.40, 0.30)
    
    node.get_logger().info("Sequence COMPLETE!")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
