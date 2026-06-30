#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time
import argparse
import sys

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from geometry_msgs.msg import Pose
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

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw

def wait_for_future(node, future, timeout_sec=None):
    start = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if future.done():
            return future.result()
        if timeout_sec and (time.time() - start) > timeout_sec:
            node.get_logger().error("Future timed out!")
            return None

class FullCleaningSequence(Node):
    def __init__(self, joint_speed=0.2, cart_speed=0.5):
        super().__init__('full_cleaning_sequence')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        self.joint_names = [
            'ur5e_shoulder_pan_joint',
            'ur5e_shoulder_lift_joint',
            'ur5e_elbow_joint',
            'ur5e_wrist_1_joint',
            'ur5e_wrist_2_joint',
            'ur5e_wrist_3_joint'
        ]
        
        self.joint_speed = joint_speed
        self.cart_speed = cart_speed

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('ur5e_base_link', 'ur5e_tool0', rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0))
            return trans
        except Exception as e:
            self.get_logger().error(f"TF Lookup Failed! Cannot get tool0 location: {e}")
            return None

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
        self.get_logger().info("Planning safe joint trajectory...")
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
        
        # MoveIt's default planner often returns trajectories that just need execution
        self.get_logger().info("Safe joint path found! Executing...")
        return self.execute_trajectory(traj)

    def execute_cartesian_path(self, waypoints):
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            return False
            
        req = GetCartesianPath.Request()
        req.header.frame_id = 'ur5e_base_link'
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

    def execute_absolute_cartesian(self, x_cm, y_cm, z_cm):
        self.get_logger().info(f"Moving Absolute X:{x_cm}cm, Y:{y_cm}cm, Z:{z_cm}cm")
        curr = self.get_current_pose()
        if not curr: return False
        
        x_table = x_cm / 100.0
        y_table = y_cm / 100.0
        z_table = z_cm / 100.0
        
        theta = math.radians(45.0)
        x_base = x_table * math.cos(theta) - y_table * math.sin(theta)
        y_base = x_table * math.sin(theta) + y_table * math.cos(theta)
        
        target = Pose()
        target.position.x = x_base
        target.position.y = y_base
        target.position.z = z_table
        target.orientation = curr.transform.rotation
        
        return self.execute_cartesian_path([target])

    def execute_reorient(self, pitch_deg, roll_deg, yaw_deg):
        self.get_logger().info(f"Rotating to Pitch:{pitch_deg}, Roll:{roll_deg}, Yaw:{yaw_deg} (wrt Z-axis table frame)")
        curr = self.get_current_pose()
        if not curr: return False
        
        start_q = curr.transform.rotation
        start_roll, start_pitch, start_yaw = euler_from_quaternion(start_q.x, start_q.y, start_q.z, start_q.w)
        
        table_offset_deg = 45.0
        target_roll = math.pi + math.radians(pitch_deg)
        target_pitch = 0.0 + math.radians(roll_deg)
        target_yaw = math.radians(yaw_deg + table_offset_deg)
        
        # Slicing the rotation into 10 waypoints to force Cartesian rigidity
        waypoints = []
        steps = 10
        
        # Shortest path math for yaw interpolation
        diff_roll = (target_roll - start_roll + math.pi) % (2 * math.pi) - math.pi
        diff_pitch = (target_pitch - start_pitch + math.pi) % (2 * math.pi) - math.pi
        diff_yaw = (target_yaw - start_yaw + math.pi) % (2 * math.pi) - math.pi
        
        for i in range(1, steps + 1):
            fraction = i / float(steps)
            r = start_roll + diff_roll * fraction
            p = start_pitch + diff_pitch * fraction
            y = start_yaw + diff_yaw * fraction
            
            new_q = quaternion_from_euler(r, p, y)
            
            target = Pose()
            target.position.x = curr.transform.translation.x
            target.position.y = curr.transform.translation.y
            target.position.z = curr.transform.translation.z
            target.orientation.x = new_q[0]
            target.orientation.y = new_q[1]
            target.orientation.z = new_q[2]
            target.orientation.w = new_q[3]
            waypoints.append(target)
        
        return self.execute_cartesian_path(waypoints)


def main(args=None):
    parser = argparse.ArgumentParser(description='Full Cleaning Sequence')
    parser.add_argument('--joint_speed', type=float, default=0.2, help='Speed factor for joint moves (0.0 to 1.0)')
    parser.add_argument('--cart_speed', type=float, default=0.5, help='Speed factor for Cartesian moves (0.0 to 1.0)')
    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:])
    
    rclpy.init(args=ros_args)
    node = FullCleaningSequence(joint_speed=parsed_args.joint_speed, cart_speed=parsed_args.cart_speed)
    
    # Wait for tf tree to populate
    time.sleep(2.0)
    
    start_joints = [
        0.07405982166528702,
        -2.548169275323385,
        -0.7227350473403931,
        1.5707674026489258,
        1.6449581384658813,
        -1.4414236408523102
    ]
    
    # Notice: the wrist 1, 2, 3 order was specified differently in the text file!
    # Text file order: pan, lift, elbow, wrist_2, wrist_3, wrist_1
    # My joint names list: pan, lift, elbow, wrist_1, wrist_2, wrist_3
    # I need to swap wrist 1 and wrist 2/3 values to match our joint names list!
    
    # Fix mapping based on text file:
    # 0 = pan
    # 1 = lift
    # 2 = elbow
    # 3 = wrist_2 = 1.5707
    # 4 = wrist_3 = 1.6449
    # 5 = wrist_1 = -1.4414
    
    # Correctly mapped joint array for ur_arm group:
    start_joints_corrected = [
        0.07405982166528702,
        -2.548169275323385,
        -0.7227350473403931,
        -1.4414236408523102, # wrist 1
        1.5707674026489258,  # wrist 2
        1.6449581384658813   # wrist 3
    ]
    
    try:
        # Step 1: Collision Aware Joint Move
        if not node.execute_joint_trajectory_safe(start_joints_corrected): raise Exception("Step 1 Failed")
        time.sleep(0.5)
        
        # Step 2: move 25 cm in -z direction to start point
        if not node.execute_absolute_cartesian(-55.0, 65.0, 0.0): raise Exception("Failed")
        time.sleep(0.5)
        
        # Define the sweep sequence: (x, y, z, required_yaw_after_move)
        # Note: We rotate *after* the sweep across X, but *before* the Y step down, as requested previously (move, stop, rotate, move).
        # Actually, let's explicitly define every single target to match the text exactly.
        
        sequence_steps = [
            # Sweep Right
            {'type': 'move', 'x': 55.0, 'y': 65.0, 'z': 0.0},
            {'type': 'rot', 'yaw': -135.0},
            {'type': 'move', 'x': 55.0, 'y': 60.0, 'z': 0.0},
            
            # Sweep Left
            {'type': 'move', 'x': -55.0, 'y': 60.0, 'z': 0.0},
            {'type': 'rot', 'yaw': -45.0},
            {'type': 'move', 'x': -55.0, 'y': 55.0, 'z': 0.0},
            
            # Sweep Right
            {'type': 'move', 'x': 55.0, 'y': 55.0, 'z': 0.0},
            {'type': 'rot', 'yaw': -135.0},
            {'type': 'move', 'x': 55.0, 'y': 50.0, 'z': 0.0},
            
            # Sweep Left
            {'type': 'move', 'x': -55.0, 'y': 50.0, 'z': 0.0},
            {'type': 'rot', 'yaw': -45.0},
            {'type': 'move', 'x': -55.0, 'y': 45.0, 'z': 0.0},
            
            # Sweep Right
            {'type': 'move', 'x': 55.0, 'y': 45.0, 'z': 0.0},
            {'type': 'rot', 'yaw': -135.0},
            {'type': 'move', 'x': 55.0, 'y': 40.0, 'z': 0.0},
            
            # Sweep Left
            {'type': 'move', 'x': -55.0, 'y': 40.0, 'z': 0.0},
            {'type': 'rot', 'yaw': -45.0},
            {'type': 'move', 'x': -55.0, 'y': 35.0, 'z': 0.0},
            
            # Sweep Right
            {'type': 'move', 'x': 55.0, 'y': 35.0, 'z': 0.0},
            {'type': 'rot', 'yaw': -135.0},
            {'type': 'move', 'x': 55.0, 'y': 30.0, 'z': 0.0},
            
            # Sweep Left (Final)
            {'type': 'move', 'x': -55.0, 'y': 30.0, 'z': 0.0},
            
            # Lift Z
            {'type': 'move', 'x': -55.0, 'y': 30.0, 'z': 25.0}
        ]
        
        for step in sequence_steps:
            if step['type'] == 'move':
                if not node.execute_absolute_cartesian(step['x'], step['y'], step['z']): raise Exception(f"Failed move to {step}")
            elif step['type'] == 'rot':
                if not node.execute_reorient(0.0, 0.0, step['yaw']): raise Exception(f"Failed rot to {step}")
            time.sleep(0.5)
        
        node.get_logger().info("SUCCESS! Cleaning sequence finished!")
        
    except Exception as e:
        node.get_logger().error(f"Sequence aborted: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
