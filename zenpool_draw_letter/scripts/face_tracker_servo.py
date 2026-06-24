#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController
import cv2
import time
import os
import urllib.request
import math

class FaceTrackerServo(Node):
    def __init__(self):
        super().__init__('face_tracker_servo')
        
        # Publishers
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.marker_pub = self.create_publisher(Marker, '/camera_mount_marker', 10)
        
        # Ensure we are in velocity control mode
        self.switch_controller("forward_velocity_controller", "scaled_joint_trajectory_controller")
        self.start_servo()
        
        # Vision Setup
        self.get_logger().info("Initializing Camera...")
        self.cap = None
        for i in range(4):
            self.cap = cv2.VideoCapture(i)
            if self.cap.isOpened():
                self.get_logger().info(f"Opened camera at /dev/video{i}")
                break
            self.cap.release()
            
        if self.cap is None or not self.cap.isOpened():
            self.get_logger().error("Could not open camera!")
            raise RuntimeError("Camera failed to open.")

        # Load Haar Cascade Model safely
        self.cascade_file = 'haarcascade_frontalface_default.xml'
        if not os.path.exists(self.cascade_file):
            url = "https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml"
            urllib.request.urlretrieve(url, self.cascade_file)
        self.face_cascade = cv2.CascadeClassifier(self.cascade_file)

        # Control Loop State
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0
        self.last_wx = 0.0
        self.last_wy = 0.0
        self.last_wz = 0.0
        
        # Timer for control loop at 30Hz (matching typical camera FPS)
        self.timer_period = 0.033 
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info("Visual Servoing Active! Tracking faces in Hybrid Mode...")

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
        self.get_logger().info("Switched to forward_velocity_controller!")

    def start_servo(self):
        client = self.create_client(Trigger, '/servo_node/start_servo')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /servo_node/start_servo...')
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("MoveIt Servo is awake!")

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
        ret, frame = self.cap.read()
        if not ret:
            return
            
        height, width, _ = frame.shape
        center_x_img = width / 2.0
        center_y_img = height / 2.0
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))
        
        target_vx = 0.0
        target_vy = 0.0
        target_vz = 0.0
        target_wx = 0.0
        target_wy = 0.0
        target_wz = 0.0

        if len(faces) > 0:
            # Lock onto the largest face (closest to camera)
            largest_face = max(faces, key=lambda rect: rect[2] * rect[3])
            x, y, w, h = largest_face
            
            # Constants
            KNOWN_WIDTH_CM = 14.0  
            FOCAL_LENGTH_PIXELS = 600.0  
            TARGET_DISTANCE_CM = 35.0  # Adjusted per user request
            
            # Z Axis: Distance Error (Linear)
            current_distance_cm = (KNOWN_WIDTH_CM * FOCAL_LENGTH_PIXELS) / w
            error_z_cm = current_distance_cm - TARGET_DISTANCE_CM
            
            # Physical Camera Calibration Offset
            # If the robot naturally tilts downwards, it means your camera is physically glued pointing slightly UP!
            # We can fix this by mathematically shifting the "center" of the image.
            # Decrease this number (e.g. -100) to make the robot hold its head HIGHER.
            TILT_OFFSET_PIXELS = -80.0 
            
            # X/Y Axis: Pixel Error from Center
            face_cx = x + w / 2.0
            face_cy = y + h / 2.0
            error_x_px = face_cx - center_x_img
            error_y_px = (face_cy - center_y_img) + TILT_OFFSET_PIXELS
            
            # --- Visual Servoing PID Controller (HYBRID MODE) ---
            # To avoid wrist singularities, we command the robot to BOTH slide and pivot.
            # This distributes the movement across all 6 joints beautifully.
            kp_ang = 0.0015 # rad/s per pixel error (Rotation)
            kp_lin = 0.0005 # m/s per pixel error (Sliding)
            kp_z = 0.015   # m/s per cm error (Distance)
            
            # Neck movement (Pan/Tilt) - Inverted based on user feedback
            target_wy = -kp_ang * error_x_px 
            target_wx = -kp_ang * error_y_px
            
            # Drone movement (Sliding) - Inverted to match the camera orientation
            target_vx = -kp_lin * error_x_px
            target_vy = -kp_lin * error_y_px
            
            # Move forward/backward to maintain 50cm 
            target_vz = kp_z * error_z_cm
            
            # Draw targeting UI
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.line(frame, (int(center_x_img), int(center_y_img)), (int(face_cx), int(face_cy)), (0, 255, 255), 2)
            cv2.putText(frame, f"Dist: {current_distance_cm:.1f} cm", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Apply Safety Caps (Increased for more flexibility/speed)
        max_v = 0.25 # m/s for linear sliding
        max_w = 0.6  # rad/s for turning
        
        # Linear Speed Cap
        v_mag = math.sqrt(target_vx**2 + target_vy**2 + target_vz**2)
        if v_mag > max_v:
            target_vx *= (max_v / v_mag)
            target_vy *= (max_v / v_mag)
            target_vz *= (max_v / v_mag)
            
        # Angular Speed Cap
        w_mag = math.sqrt(target_wx**2 + target_wy**2)
        if w_mag > max_w:
            target_wx *= (max_w / w_mag)
            target_wy *= (max_w / w_mag)
            
        # Apply Acceleration Limiter (Slew Rate) to prevent motor jerking
        max_a_lin = 0.5 # m/s^2
        max_a_ang = 1.0 # rad/s^2
        
        self.last_vx = self.limit_accel(target_vx, self.last_vx, max_a_lin, self.timer_period)
        self.last_vy = self.limit_accel(target_vy, self.last_vy, max_a_lin, self.timer_period)
        self.last_vz = self.limit_accel(target_vz, self.last_vz, max_a_lin, self.timer_period)
        
        self.last_wx = self.limit_accel(target_wx, self.last_wx, max_a_ang, self.timer_period)
        self.last_wy = self.limit_accel(target_wy, self.last_wy, max_a_ang, self.timer_period)
        self.last_wz = 0.0 # No roll
        
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
        
        # Publish Camera Holder Marker to RViz
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "ur5e_tool0"
        marker.ns = "camera_mount"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = "package://my_robot_cell_description/meshes/CamHolder.stl"
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.02 # Slightly offset it past the baseplate if needed
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.001 # STL export is usually in mm, scale to meters
        marker.scale.y = 0.001
        marker.scale.z = 0.001
        marker.color.a = 0.9 # Slightly transparent
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.8 # Blue color to easily distinguish it
        self.marker_pub.publish(marker)
        
        # Display GUI
        cv2.drawMarker(frame, (int(center_x_img), int(center_y_img)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.imshow("Visual Servoing", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerServo()
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
        
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
