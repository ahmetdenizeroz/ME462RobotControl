#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import math

from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArucoTrackerServo(Node):
    def __init__(self):
        super().__init__('aruco_tracker_servo')
        
        # Control Loop State
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0
        self.last_wx = 0.0
        self.last_wy = 0.0
        self.last_wz = 0.0
        
        # PID State Variables
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_pitch = 0.0
        self.integral_yaw = 0.0
        
        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_err_pitch = 0.0
        self.prev_err_yaw = 0.0
        
        # EMA Filter States
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_pitch = 0.0
        self.filtered_yaw = 0.0
        self.first_msg = True
        
        self.last_callback_time = self.get_clock().now()
        
        # The target velocities (updated by the callback)
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wx = 0.0
        self.target_wy = 0.0
        
        # Timestamp to track if we lost the ArUco
        self.last_seen_time = self.get_clock().now()
        
        # Node State Machine
        self.state = 'INIT' # States: 'INIT', 'SERVO'
        
        # Publishers and Subscribers
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.offset_sub = self.create_subscription(Twist, '/gripper/aruco_offset', self.offset_callback, 10)
        
        # Action Client for Autonomous Homing
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        # Timer for control loop at 50Hz
        self.timer_period = 0.02 
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        # Timer to trigger the initialization homing sequence once the node is spinning
        self.init_timer = self.create_timer(1.0, self.trigger_initialization)
        
        self.get_logger().info("ArucoTrackerServo node started. Waiting 1 second to begin homing sequence...")

    def trigger_initialization(self):
        self.init_timer.cancel() # Only run once
        self.get_logger().info("Connecting to Joint Trajectory Controller...")
        
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Failed to connect to /joint_trajectory_controller/follow_joint_trajectory! Cannot home the robot.")
            return
            
        self.get_logger().info("Homing robot to predefined Cartesian tracking start pose...")
        
        # Create Goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'ur5e_shoulder_pan_joint',
            'ur5e_shoulder_lift_joint',
            'ur5e_elbow_joint',
            'ur5e_wrist_1_joint',
            'ur5e_wrist_2_joint',
            'ur5e_wrist_3_joint'
        ]
        
        # Create a single trajectory point reaching the goal in 4.0 seconds
        point = JointTrajectoryPoint()
        point.positions = [-0.785398, -0.785398, -1.570796, -2.356194, 1.570796, 0.0]
        point.time_from_start = Duration(sec=4, nanosec=0)
        
        goal_msg.trajectory.points = [point]
        
        # Send Goal Asynchronously
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Homing trajectory was REJECTED by the controller!")
            return
            
        self.get_logger().info("Homing trajectory accepted. Moving robot (ETA 4 seconds)...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.homing_complete_callback)

    def homing_complete_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Homing movement finished with error code: {result.error_code}")
        
        self.get_logger().info("Activating MoveIt Servo for ArUco Tracking...")
        # Ensure we are in position control mode
        self.switch_controller("forward_position_controller", "joint_trajectory_controller")
        self.start_servo()
        
        self.state = 'SERVO'
        self.get_logger().info("ArUco Visual Servoing Active! Tracking in Hybrid Mode with Cartesian sliding...")

    def switch_controller(self, start, stop):
        client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for switch_controller...')
        req = SwitchController.Request()
        req.start_controllers = [start]
        req.stop_controllers = [stop]
        req.strictness = SwitchController.Request.BEST_EFFORT
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().ok:
            self.get_logger().info("Switched to forward_position_controller successfully!")
        else:
            self.get_logger().error("FAILED to switch to forward_position_controller! Is it loaded in the UR driver?")

    def start_servo(self):
        client = self.create_client(Trigger, '/servo_node/start_servo')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /servo_node/start_servo...')
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("MoveIt Servo is awake!")

    def offset_callback(self, msg: Twist):
        if self.state != 'SERVO':
            return
            
        current_time = self.get_clock().now()
        dt = (current_time - self.last_callback_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 0.033 # fallback for very first run or simultaneous messages
            
        self.last_callback_time = current_time
        self.last_seen_time = current_time
        
        raw_x = msg.linear.x
        raw_y = msg.linear.y
        raw_pitch = msg.angular.x
        raw_yaw = msg.angular.y
        
        # --- Exponential Moving Average (EMA) Low-Pass Filter ---
        # alpha determines the smoothing amount. 
        # 1.0 = No smoothing (raw data). 0.1 = Very smooth (but adds lag).
        alpha = 0.3 
        
        if self.first_msg:
            self.filtered_x = raw_x
            self.filtered_y = raw_y
            self.filtered_pitch = raw_pitch
            self.filtered_yaw = raw_yaw
            self.first_msg = False
        else:
            self.filtered_x = alpha * raw_x + (1.0 - alpha) * self.filtered_x
            self.filtered_y = alpha * raw_y + (1.0 - alpha) * self.filtered_y
            self.filtered_pitch = alpha * raw_pitch + (1.0 - alpha) * self.filtered_pitch
            self.filtered_yaw = alpha * raw_yaw + (1.0 - alpha) * self.filtered_yaw
            
        error_x_px = self.filtered_x
        error_y_px = self.filtered_y
        error_pitch = self.filtered_pitch
        error_yaw = self.filtered_yaw
        
        # Calculate 2D Euclidean distance of the pixel error
        distance_xy = math.sqrt(error_x_px**2 + error_y_px**2)
        
        # Apply 5-pixel deadzone (hysteresis) to prevent jitter
        if abs(error_x_px) <= 5.0:
            error_x_px = 0.0
        if abs(error_y_px) <= 5.0:
            error_y_px = 0.0
            
        # Apply 5-degree deadzone (~0.087 rad) to pitch/yaw to prevent jitter
        if abs(error_pitch) <= 0.087:
            error_pitch = 0.0
        if abs(error_yaw) <= 0.087:
            error_yaw = 0.0
            
        # Invert axes because the camera is mounted upside down
        error_x_px = error_x_px
        error_y_px = -error_y_px
        error_pitch = -error_pitch
            
        # --- Visual Servoing PID Controller ---
        # Positional PID Gains
        # Tuned to push the slow zero to -0.5 for fast centering, 
        # and increased Kd to provide phase lead against the 100ms network latency.
        kp_lin = 0.01
        ki_lin = 0.000002
        kd_lin = 0.0002
        
        # Angular PID Gains
        # Tuned to eliminate the 50-second slow tail, pushing the slow zero to -0.5.
        # Kd is slightly reduced to prevent the fast zero from amplifying high-frequency noise.
        kp_ang = 0.5
        ki_ang = 0.25
        kd_ang = 0.025
        
        # Integration (Cartesian only)
        self.integral_x += error_x_px * dt
        self.integral_y += error_y_px * dt
        
        # Integral Anti-Windup
        self.integral_x = max(-2000.0, min(self.integral_x, 2000.0))
        self.integral_y = max(-2000.0, min(self.integral_y, 2000.0))
        
        # Derivative (Cartesian only)
        deriv_x = (error_x_px - self.prev_err_x) / dt
        deriv_y = (error_y_px - self.prev_err_y) / dt
        
        # Save previous errors
        self.prev_err_x = error_x_px
        self.prev_err_y = error_y_px
        
        # Drone movement (Sliding to keep centered)
        # Note: Y is explicitly inverted to match physical layout
        self.target_vx = - (kp_lin * error_x_px + ki_lin * self.integral_x + kd_lin * deriv_x)
        self.target_vy = + (kp_lin * error_y_px + ki_lin * self.integral_y + kd_lin * deriv_y)
        self.target_vz = 0.0
        
        # Disable angular alignment per user request to maintain Cartesian parallel constraint
        self.target_wx = 0.0
        self.target_wy = 0.0
        self.target_wz = 0.0
        
        # Apply Safety Caps
        max_v = 0.25 # m/s
        max_w = 0.6  # rad/s
        
        v_mag = math.sqrt(self.target_vx**2 + self.target_vy**2)
        if v_mag > max_v:
            self.target_vx *= (max_v / v_mag)
            self.target_vy *= (max_v / v_mag)
            
        w_mag = math.sqrt(self.target_wx**2 + self.target_wy**2)
        if w_mag > max_w:
            self.target_wx *= (max_w / w_mag)
            self.target_wy *= (max_w / w_mag)

    def limit_accel(self, target, last, max_a, dt):
        """ Smooth acceleration profile (Slew Rate Limiter) """
        dv = target - last
        max_dv = max_a * dt
        if dv > max_dv:
            return last + max_dv
        elif dv < -max_dv:
            return last - max_dv
        return target

    def control_loop(self):
        if self.state != 'SERVO':
            return
            
        # Fail-safe: If we haven't seen the ArUco for 0.5 seconds, stop the robot smoothly!
        if (self.get_clock().now() - self.last_seen_time).nanoseconds > 500_000_000:
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_wx = 0.0
            self.target_wy = 0.0
            
        # Apply Acceleration Limiter (Slew Rate) to prevent motor jerking
        max_a_lin = 0.5 # m/s^2
        max_a_ang = 1.0 # rad/s^2
        
        self.last_vx = self.limit_accel(self.target_vx, self.last_vx, max_a_lin, self.timer_period)
        self.last_vy = self.limit_accel(self.target_vy, self.last_vy, max_a_lin, self.timer_period)
        self.last_vz = 0.0
        
        self.last_wx = self.limit_accel(self.target_wx, self.last_wx, max_a_ang, self.timer_period)
        self.last_wy = self.limit_accel(self.target_wy, self.last_wy, max_a_ang, self.timer_period)
        self.last_wz = 0.0
        
        # Publish Twist
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "ur5e_tool0" 
        
        twist_msg.twist.linear.x = self.last_vx
        twist_msg.twist.linear.y = self.last_vy
        twist_msg.twist.linear.z = self.last_vz
        
        twist_msg.twist.angular.x = self.last_wx
        twist_msg.twist.angular.y = self.last_wy
        twist_msg.twist.angular.z = self.last_wz
        
        self.twist_pub.publish(twist_msg)
        
        # Only print if we are actually requesting a non-zero velocity
        if abs(self.last_vx) > 0.001 or abs(self.last_vy) > 0.001 or abs(self.last_wx) > 0.001 or abs(self.last_wy) > 0.001:
            self.get_logger().info(f"Sending Twist - VX: {self.last_vx:.3f}, VY: {self.last_vy:.3f}, WX: {self.last_wx:.3f}, WY: {self.last_wy:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on exit
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = "ur5e_tool0"
        node.twist_pub.publish(stop_msg)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
