import os
import re

scripts = [
    'take_one.py', 'drop_one.py',
    'take_two.py', 'drop_two.py',
    'take_three.py', 'drop_three.py'
]

directory = '/home/deniz/generalrobotcontrol/src/ME462RobotControl/zenpool_draw_letter/scripts'

manhattan_func = '''
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
'''

for script in scripts:
    path = os.path.join(directory, script)
    with open(path, 'r') as f:
        content = f.read()
    
    # 1. Clean imports
    content = content.replace("from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetMotionPlan\\nfrom moveit_msgs.msg import Constraints, JointConstraint",
                              "from moveit_msgs.srv import GetCartesianPath, GetPositionFK")
    content = content.replace("from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetMotionPlan\nfrom moveit_msgs.msg import Constraints, JointConstraint",
                              "from moveit_msgs.srv import GetCartesianPath, GetPositionFK")

    # 2. Clean client
    content = content.replace("self.fk_client = self.create_client(GetPositionFK, '/compute_fk')\\n        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')",
                              "self.fk_client = self.create_client(GetPositionFK, '/compute_fk')")
    content = content.replace("self.fk_client = self.create_client(GetPositionFK, '/compute_fk')\n        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')",
                              "self.fk_client = self.create_client(GetPositionFK, '/compute_fk')")

    # 3. Replace safe function with manhattan func
    # Using regex to remove the entire safe block up to `def execute_joint_trajectory(`
    pattern = re.compile(r'    def execute_joint_trajectory_safe\(self, target_joints\):.*?    def execute_joint_trajectory\(self, target_joints, sec=3\):', re.DOTALL)
    if 'execute_manhattan_to_joints' not in content:
        content = pattern.sub(manhattan_func + '\n    def execute_joint_trajectory(self, target_joints, sec=3):', content)

    # 4. Replace main calls
    content = content.replace("execute_joint_trajectory_safe(node.tool_take_anchor)", "execute_manhattan_to_joints(node.tool_take_anchor)")
    content = content.replace("execute_joint_trajectory_safe(node.tool_drop_anchor)", "execute_manhattan_to_joints(node.tool_drop_anchor)")
    
    with open(path, 'w') as f:
        f.write(content)
        
print("Manhattan patch applied successfully!")
