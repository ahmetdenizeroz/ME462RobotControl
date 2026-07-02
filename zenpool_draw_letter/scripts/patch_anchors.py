import os
import re

scripts = [
    'take_one.py', 'drop_one.py',
    'take_two.py', 'drop_two.py',
    'take_three.py', 'drop_three.py'
]

directory = '/home/deniz/generalrobotcontrol/src/ME462RobotControl/zenpool_draw_letter/scripts'

safe_func = '''
    def execute_joint_trajectory_safe(self, target_joints):
        self.get_logger().info("Planning safe joint trajectory to target...")
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Motion Plan service offline")
            return False
            
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = 'ur_arm'
        req.motion_plan_request.num_planning_attempts = 5
        req.motion_plan_request.allowed_planning_time = 5.0
        
        speed_factor = 0.4
        req.motion_plan_request.max_velocity_scaling_factor = speed_factor
        req.motion_plan_request.max_acceleration_scaling_factor = speed_factor
        
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
        
        # Scale speed safely
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
                
        # We can reuse execute_cartesian_trajectory because it just sends a FollowJointTrajectory goal!
        return self.execute_cartesian_trajectory(traj)

    def execute_joint_trajectory(self, target_joints, sec=3):'''

for script in scripts:
    path = os.path.join(directory, script)
    with open(path, 'r') as f:
        content = f.read()
    
    # 1. Imports
    if 'GetMotionPlan' not in content:
        content = content.replace('from moveit_msgs.srv import GetCartesianPath, GetPositionFK', 
                                  'from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetMotionPlan\\nfrom moveit_msgs.msg import Constraints, JointConstraint')
    
    # 2. Client
    if 'self.plan_client =' not in content:
        content = content.replace("self.fk_client = self.create_client(GetPositionFK, '/compute_fk')",
                                  "self.fk_client = self.create_client(GetPositionFK, '/compute_fk')\\n        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')")
                                  
    # 3. Function Injection
    if 'execute_joint_trajectory_safe' not in content:
        content = content.replace('    def execute_joint_trajectory(self, target_joints, sec=3):', safe_func)
        
    # 4. Main loop call replacement
    content = re.sub(r'execute_joint_trajectory\(node\.tool_take_anchor, sec=\d+\)', 'execute_joint_trajectory_safe(node.tool_take_anchor)', content)
    content = re.sub(r'execute_joint_trajectory\(node\.tool_drop_anchor, sec=\d+\)', 'execute_joint_trajectory_safe(node.tool_drop_anchor)', content)
    
    with open(path, 'w') as f:
        f.write(content)
print('Patching complete!')
