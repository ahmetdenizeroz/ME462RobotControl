#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import tf2_ros

class TakePapisNode(Node):
    def __init__(self):
        super().__init__('take_papis_node')
        
        self.joint_names = [
            'ur5e_shoulder_pan_joint',
            'ur5e_shoulder_lift_joint',
            'ur5e_elbow_joint',
            'ur5e_wrist_1_joint',
            'ur5e_wrist_2_joint',
            'ur5e_wrist_3_joint'
        ]
        
        self.traj_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.pub_tool = self.create_publisher(String, '/tool_manager/command', 10)
        
        self.current_joint_state = None
        self.js_sub = self.create_subscription(JointState, '/joint_states', self.js_callback, 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Wait for tf tree and joint states to build up
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.5)

    def js_callback(self, msg):
        self.current_joint_state = msg

    def spin_for(self, seconds):
        start = time.time()
        while time.time() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_sequence(self):
        self.get_logger().info("Starting Take Papis Sequence...")
        
        self.get_logger().info("Move 1: Take Papis Anchor (Joint Space)")
        self.move_joints([0.427368, -1.413317, -1.744605, -1.554056, 1.570809, 3.088430])
        
        self.get_logger().info("Move 2 (Joint Space)")
        self.move_joints([0.792035, -2.174871, -2.322407, -0.214535, -1.566356, 3.083950])
        
        self.get_logger().info("Move 3 (Joint Space)")
        self.move_joints([1.179408, -2.243419, -2.314682, -0.153784, -1.566291, 3.083528])
        
        self.get_logger().info("Move 4 (Joint Space)")
        self.move_joints([1.132618, -2.758818, -1.213140, -0.739929, -1.570631, 2.794143])
        
        self.get_logger().info("Move 5: Cartesian Pool Frame [-0.759, -0.133, 0.231]")
        self.execute_absolute_cartesian('pool_actual', -0.759, -0.133, 0.231)
        
        self.get_logger().info("Move 6: Cartesian Pool Frame [-0.459, -0.133, 0.231]")
        self.execute_absolute_cartesian('pool_actual', -0.459, -0.133, 0.231)
        
        self.get_logger().info("====================================")
        self.get_logger().info("    TAKE PAPIS SEQUENCE COMPLETE!   ")
        self.get_logger().info("====================================")

    def execute_absolute_cartesian(self, frame_id, x, y, z):
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cartesian Server offline!")
            return False
            
        try:
            trans = self.tf_buffer.lookup_transform(frame_id, 'ur5e_tool0', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f"TF Lookup Failed for {frame_id}! {e}")
            return False
            
        target_pose = Pose()
        target_pose.position.x = float(x)
        target_pose.position.y = float(y)
        target_pose.position.z = float(z)
        target_pose.orientation = trans.transform.rotation
        
        req_cart = GetCartesianPath.Request()
        req_cart.header.frame_id = frame_id
        req_cart.group_name = 'ur_arm'
        req_cart.waypoints = [target_pose]
        req_cart.max_step = 0.01
        req_cart.avoid_collisions = True
        
        future_cart = self.cartesian_client.call_async(req_cart)
        rclpy.spin_until_future_complete(self, future_cart)
        
        res_cart = future_cart.result()
        if res_cart.fraction < 0.99:
            self.get_logger().error(f"Cartesian Path aborted due to collision! Fraction: {res_cart.fraction*100:.1f}%")
            return False
            
        self.execute_trajectory(res_cart.solution.joint_trajectory)
        return True

    def get_current_joints(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        if not self.current_joint_state:
            return None
        current = []
        for name in self.joint_names:
            try:
                idx = self.current_joint_state.name.index(name)
                current.append(self.current_joint_state.position[idx])
            except ValueError:
                return None
        return current

    def move_joints(self, joints):
        self.get_logger().info(f"Moving to joints: {joints}")
        
        if not self.traj_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory Server offline!")
            return False
            
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        from trajectory_msgs.msg import JointTrajectoryPoint
        point = JointTrajectoryPoint()
        point.positions = joints
        
        # Calculate dynamic time based on 0.3 speed limit
        curr = self.get_current_joints()
        if curr:
            max_diff = max(abs(t - c) for t, c in zip(joints, curr))
            max_vel_rads = 3.14 * 0.3  # UR5e max joint speed is ~3.14 rad/s
            req_time = (max_diff / max_vel_rads) + 1.0 # +1.0s buffer for acceleration
            
            calc_sec = int(req_time)
            calc_nanosec = int((req_time - calc_sec) * 1e9)
            
            if req_time < 2.0:
                point.time_from_start.sec = 2
                point.time_from_start.nanosec = 0
            else:
                point.time_from_start.sec = calc_sec
                point.time_from_start.nanosec = calc_nanosec
                
            self.get_logger().info(f"Calculated Joint move time: {req_time:.2f} seconds for max distance {max_diff:.2f} rad")
        else:
            self.get_logger().warn("No joint states! Falling back to safe 8 second move.")
            point.time_from_start.sec = 8
            point.time_from_start.nanosec = 0
            
        goal_msg.trajectory.points = [point]
        
        future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Joint trajectory goal rejected!")
            return False
            
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
            
        self.get_logger().info("Joint movement completed.")
        return True

    def execute_trajectory(self, traj):
        if len(traj.points) > 1 and traj.points[-1].time_from_start.sec == 0 and traj.points[-1].time_from_start.nanosec == 0:
            cumulative_sec = 0.0
            for point in traj.points:
                cumulative_sec += 0.05
                point.time_from_start.sec = int(cumulative_sec)
                point.time_from_start.nanosec = int((cumulative_sec - int(cumulative_sec)) * 1e9)
                
        speed_factor = 0.2
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
                
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        
        traj_future = self.traj_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, traj_future)
        
        goal_handle = traj_future.result()
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)

def main(args=None):
    rclpy.init(args=args)
    node = TakePapisNode()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
