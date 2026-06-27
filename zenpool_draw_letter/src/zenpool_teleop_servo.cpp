#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <mutex>

class TeleopServoNode : public rclcpp::Node {
public:
    TeleopServoNode() : Node("zenpool_teleop_servo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("teleop_marker", this);
    }

    void init(rclcpp::Node::SharedPtr node_ptr) {
        // Correctly pass the node to the TransformListener so it spins with our executor
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_ptr);

        RCLCPP_INFO(this->get_logger(), "Waiting for TF tree to populate...");
        while (rclcpp::ok() && !tf_buffer_->canTransform("ur5e_base_link", "ur5e_tool0", tf2::TimePointZero, tf2::durationFromSec(1.0))) {
            RCLCPP_INFO(this->get_logger(), "Still waiting for ur5e_base_link -> ur5e_tool0 transform...");
        }

        if (!rclcpp::ok()) return;

        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("ur5e_base_link", "ur5e_tool0", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not get initial robot pose! %s", ex.what());
            return;
        }

        target_pose_.position.x = transformStamped.transform.translation.x;
        target_pose_.position.y = transformStamped.transform.translation.y;
        target_pose_.position.z = transformStamped.transform.translation.z;
        target_pose_.orientation = transformStamped.transform.rotation;

        createInteractiveMarker();

        // Ensure controllers and servo are ready
        switchController("forward_position_controller", "joint_trajectory_controller");
        startServo();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&TeleopServoNode::controlLoop, this));
        RCLCPP_INFO(this->get_logger(), "Teleop Node Ready! You can now drag the 'robot_target' marker in RViz.");
    }

private:
    void switchController(const std::string& start, const std::string& stop) {
        auto client = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Waiting for switch_controller service...");
        }
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->start_controllers = {start};
        request->stop_controllers = {stop};
        request->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        
        auto result_future = client->async_send_request(request);
        if (result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Successfully switched to forward_velocity_controller!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for switch_controller response! The controller might not have switched!");
        }
    }

    void startServo() {
        auto client = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) return;
            RCLCPP_INFO(this->get_logger(), "Waiting for /servo_node/start_servo service...");
        }
        
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result_future = client->async_send_request(request);
        if (result_future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "MoveIt Servo wake-up call confirmed!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for start_servo response!");
        }
    }

    void createInteractiveMarker() {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "ur5e_base_link";
        int_marker.header.stamp = rclcpp::Time(0); // Prevents timestamp expiration
        int_marker.name = "robot_target";
        int_marker.description = "Drag to move the robot";
        int_marker.scale = 0.25;

        int_marker.pose = target_pose_;

        // Create a central sphere so it's easily visible
        visualization_msgs::msg::Marker box_marker;
        box_marker.type = visualization_msgs::msg::Marker::SPHERE;
        box_marker.scale.x = 0.05;
        box_marker.scale.y = 0.05;
        box_marker.scale.z = 0.05;
        box_marker.color.r = 1.0;
        box_marker.color.g = 1.0;
        box_marker.color.b = 0.0;
        box_marker.color.a = 1.0;
        box_marker.pose.orientation.w = 1.0; // Valid quaternion required!

        visualization_msgs::msg::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back(box_marker);
        int_marker.controls.push_back(box_control);

        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;

        tf2::Quaternion q;

        // X-axis control (Red)
        q.setRPY(0, 0, 0); // No rotation needed for X? Wait, default points to X!
        // Actually, for X-axis rotation, we rotate 90 degrees around X? No, InteractiveMarkers use specific orientations to define their axes.
        // The default axis is X. To get Z, we rotate Y by 90 degrees.
        
        q.setRPY(0.0, M_PI/2.0, 0.0);
        control.orientation.w = q.w(); control.orientation.x = q.x(); control.orientation.y = q.y(); control.orientation.z = q.z();
        control.name = "rotate_z"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        q.setRPY(0.0, 0.0, M_PI/2.0);
        control.orientation.w = q.w(); control.orientation.x = q.x(); control.orientation.y = q.y(); control.orientation.z = q.z();
        control.name = "rotate_y"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        q.setRPY(0.0, 0.0, 0.0);
        control.orientation.w = q.w(); control.orientation.x = q.x(); control.orientation.y = q.y(); control.orientation.z = q.z();
        control.name = "rotate_x"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x"; control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        server_->insert(int_marker);
        server_->setCallback(int_marker.name, std::bind(&TeleopServoNode::processFeedback, this, std::placeholders::_1));
        server_->applyChanges();
    }

    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            target_pose_ = feedback->pose;
        }
    }

    void controlLoop() {
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_->lookupTransform("ur5e_base_link", "ur5e_tool0", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            return;
        }

        std::lock_guard<std::mutex> lock(pose_mutex_);

        double ex = target_pose_.position.x - transformStamped.transform.translation.x;
        double ey = target_pose_.position.y - transformStamped.transform.translation.y;
        double ez = target_pose_.position.z - transformStamped.transform.translation.z;

        Eigen::Quaterniond q_target(target_pose_.orientation.w, target_pose_.orientation.x, target_pose_.orientation.y, target_pose_.orientation.z);
        Eigen::Quaterniond q_current(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
        
        Eigen::Quaterniond q_error = q_target * q_current.inverse();
        Eigen::AngleAxisd angle_axis_error(q_error);
        Eigen::Vector3d angular_error = angle_axis_error.axis() * angle_axis_error.angle();

        double kp_linear = 3.0;
        double kp_angular = 2.0;

        double target_vx = kp_linear * ex;
        double target_vy = kp_linear * ey;
        double target_vz = kp_linear * ez;
        double target_wx = kp_angular * angular_error.x();
        double target_wy = kp_angular * angular_error.y();
        double target_wz = kp_angular * angular_error.z();

        // Safety Caps
        double max_v = 0.3; // m/s
        double current_target_v = sqrt(pow(target_vx, 2) + pow(target_vy, 2) + pow(target_vz, 2));
        if (current_target_v > max_v) {
            target_vx *= (max_v / current_target_v);
            target_vy *= (max_v / current_target_v);
            target_vz *= (max_v / current_target_v);
        }

        double max_w = 0.5; // rad/s
        double current_target_w = sqrt(pow(target_wx, 2) + pow(target_wy, 2) + pow(target_wz, 2));
        if (current_target_w > max_w) {
            target_wx *= (max_w / current_target_w);
            target_wy *= (max_w / current_target_w);
            target_wz *= (max_w / current_target_w);
        }

        // Smooth Acceleration (Slew Rate Limiting)
        double dt = 0.002; // 500Hz loop to match native UR5e hardware
        double max_a_lin = 0.3; // m/s^2 - Low acceleration for very smooth motion
        double max_a_ang = 0.5; // rad/s^2

        auto limit_accel = [](double target, double& last, double max_a, double dt_sec) {
            double dv = target - last;
            double max_dv = max_a * dt_sec;
            if (dv > max_dv) last += max_dv;
            else if (dv < -max_dv) last -= max_dv;
            else last = target;
            return last;
        };

        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "ur5e_base_link";

        twist_msg.twist.linear.x = limit_accel(target_vx, last_vx_, max_a_lin, dt);
        twist_msg.twist.linear.y = limit_accel(target_vy, last_vy_, max_a_lin, dt);
        twist_msg.twist.linear.z = limit_accel(target_vz, last_vz_, max_a_lin, dt);
        twist_msg.twist.angular.x = limit_accel(target_wx, last_wx_, max_a_ang, dt);
        twist_msg.twist.angular.y = limit_accel(target_wy, last_wy_, max_a_ang, dt);
        twist_msg.twist.angular.z = limit_accel(target_wz, last_wz_, max_a_ang, dt);

        twist_pub_->publish(twist_msg);

        // Force RViz to see the marker by broadcasting it every 0.5 seconds
        static int heartbeat = 0;
        if (heartbeat++ % 250 == 0) {
            server_->setPose("robot_target", target_pose_);
            server_->applyChanges();
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;

    geometry_msgs::msg::Pose target_pose_;
    std::mutex pose_mutex_;

    // Store previous velocities for acceleration smoothing
    double last_vx_ = 0.0, last_vy_ = 0.0, last_vz_ = 0.0;
    double last_wx_ = 0.0, last_wy_ = 0.0, last_wz_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopServoNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    node->init(node);

    spinner.join();
    rclcpp::shutdown();
    return 0;
}
