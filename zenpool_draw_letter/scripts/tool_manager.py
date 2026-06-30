#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import struct
import math
import os

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject, AllowedCollisionMatrix, AllowedCollisionEntry

def quaternion_from_euler(roll, pitch, yaw):
    """Dependency-free Euler to Quaternion converter."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return [
        sr * cp * cy - cr * sp * sy, # x
        cr * sp * cy + sr * cp * sy, # y
        cr * cp * sy - sr * sp * cy, # z
        cr * cp * cy + sr * sp * sy  # w
    ]

def load_stl_to_mesh(file_path, scale=0.001):
    """Dependency-free binary STL parser to convert CAD to ROS shape_msgs/Mesh"""
    mesh_msg = Mesh()
    try:
        with open(file_path, 'rb') as f:
            header = f.read(80)
            num_triangles = struct.unpack('<I', f.read(4))[0]
            
            for i in range(num_triangles):
                f.read(12) # skip normal
                v1 = struct.unpack('<fff', f.read(12))
                v2 = struct.unpack('<fff', f.read(12))
                v3 = struct.unpack('<fff', f.read(12))
                f.read(2) # skip attribute
                
                idx = len(mesh_msg.vertices)
                p1 = Point(x=v1[0]*scale, y=v1[1]*scale, z=v1[2]*scale)
                p2 = Point(x=v2[0]*scale, y=v2[1]*scale, z=v2[2]*scale)
                p3 = Point(x=v3[0]*scale, y=v3[1]*scale, z=v3[2]*scale)
                
                mesh_msg.vertices.extend([p1, p2, p3])
                
                tri = MeshTriangle()
                tri.vertex_indices = [idx, idx+1, idx+2]
                mesh_msg.triangles.append(tri)
                
        return mesh_msg
    except Exception as e:
        print(f"Failed to load STL {file_path}: {e}")
        return None

class ToolManagerNode(Node):
    def __init__(self):
        super().__init__('tool_manager_node')
        
        self.pub_scene = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.sub_cmd = self.create_subscription(String, '/tool_manager/command', self.cmd_callback, 10)
        
        try:
            pkg_path = get_package_share_directory('my_robot_cell_description')
            base_path = os.path.join(pkg_path, 'meshes', 'end_effectors') + '/'
        except Exception:
            self.get_logger().error("Could not find package share dir. Ensure workspace is sourced.")
            base_path = ""
        
        # Hardcoded Attach Offsets (Extracted mathematically from your URDF)
        self.tools = {
            "tutankhamun": {
                "id": "tutankhamun_tool",
                "mesh_path": base_path + "tutan-khamun.stl",
                "toolbox_xyz": [0.539, 0.329, 0.242], 
                "toolbox_rpy": [0.0, 0.0, 2.357 + math.pi],
                "attach_xyz": [0.0, 0.0, -0.01], 
                "attach_rpy": [0.0, math.pi, 0.0],
                "scale": 0.001,
                "mesh_msg": None
            },
            "cleaner": {
                "id": "cleaner_tool",
                "mesh_path": base_path + "CleanerTool.stl",
                "toolbox_xyz": [0.370, 0.527, 0.247], 
                "toolbox_rpy": [0.0, 0.0, 2.356 + math.pi],
                "attach_xyz": [0.0, 0.0, -0.01], 
                "attach_rpy": [0.0, math.pi, 0.0],
                "scale": 1.0,
                "mesh_msg": None
            },
            "pen": {
                "id": "pen_tool",
                "mesh_path": base_path + "PenTool.stl",
                "toolbox_xyz": [0.290, 0.617, 0.247], 
                "toolbox_rpy": [0.0, 0.0, 2.357 + math.pi],
                "attach_xyz": [0.0, 0.0, -0.01], 
                "attach_rpy": [0.0, math.pi, 0.0],
                "scale": 1.0,
                "mesh_msg": None
            }
        }
        
        for name, data in self.tools.items():
            self.get_logger().info(f"Parsing binary STL mesh for {name}...")
            data["mesh_msg"] = load_stl_to_mesh(data["mesh_path"], scale=data.get("scale", 0.001))
            
        self.get_logger().info("Tool Manager Ready! Waiting 2 seconds then spawning tools...")
        self.timer = self.create_timer(2.0, self.spawn_initial_tools)
        
    def spawn_initial_tools(self):
        self.timer.cancel()
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        
        for name, tool in self.tools.items():
            if tool["mesh_msg"] is not None:
                aco = AttachedCollisionObject()
                aco.link_name = "pool"
                aco.object.id = tool["id"] + "_toolbox"
                aco.object.header.frame_id = "ur5e_base_link"
                aco.object.meshes = [tool["mesh_msg"]]
                aco.object.mesh_poses = [self.build_pose(tool["toolbox_xyz"], tool["toolbox_rpy"])]
                aco.object.operation = CollisionObject.ADD
                aco.touch_links = ["pool", "tc_arm_side", "ur5e_wrist_3_link", "ur5e_wrist_2_link", "ur5e_wrist_1_link"]
                scene_msg.robot_state.attached_collision_objects.append(aco)
                
        self.pub_scene.publish(scene_msg)
        self.get_logger().info("Successfully bundled and attached all tools to the toolbox!")
            
    def cmd_callback(self, msg: String):
        parts = msg.data.lower().strip().split()
        if len(parts) != 2:
            self.get_logger().error("Command must be 'attach <tool>' or 'detach <tool>'")
            return
            
        action = parts[0]
        tool_name = parts[1]
        
        if tool_name not in self.tools:
            self.get_logger().error(f"Unknown tool: '{tool_name}'. Available: {list(self.tools.keys())}")
            return
            
        if action == "attach":
            self.attach_tool_to_robot(tool_name)
        elif action == "detach":
            self.drop_tool_in_toolbox(tool_name)
        else:
            self.get_logger().error("Unknown action. Use 'attach' or 'detach'")

    def build_pose(self, xyz, rpy):
        p = Pose()
        p.position.x = xyz[0]
        p.position.y = xyz[1]
        p.position.z = xyz[2]
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        return p

    def drop_tool_in_toolbox(self, tool_name):
        tool = self.tools[tool_name]
        
        if tool["mesh_msg"] is None:
            self.get_logger().error(f"Cannot drop {tool_name}: Mesh was not successfully loaded!")
            return
            
        import time
        
        # --- MESSAGE 1: Detach from robot and kill ghost ---
        scene_msg1 = PlanningScene()
        scene_msg1.is_diff = True
        scene_msg1.robot_state.is_diff = True
        
        detach_obj = AttachedCollisionObject()
        detach_obj.object.id = tool["id"] + "_attached"
        detach_obj.link_name = "tc_arm_side"
        detach_obj.object.operation = CollisionObject.REMOVE
        scene_msg1.robot_state.attached_collision_objects.append(detach_obj)
        
        ghost_killer = CollisionObject()
        ghost_killer.id = tool["id"] + "_attached"
        ghost_killer.operation = CollisionObject.REMOVE
        scene_msg1.world.collision_objects.append(ghost_killer)
        
        self.pub_scene.publish(scene_msg1)
        
        # Give MoveIt 100ms to process the deletion before we add it back
        time.sleep(0.1)
        
        # --- MESSAGE 2: Attach to toolbox ---
        scene_msg2 = PlanningScene()
        scene_msg2.is_diff = True
        scene_msg2.robot_state.is_diff = True
        
        co = AttachedCollisionObject()
        co.link_name = "pool"
        co.object.id = tool["id"] + "_toolbox"
        co.object.header.frame_id = "ur5e_base_link"
        co.object.meshes = [tool["mesh_msg"]]
        co.object.mesh_poses = [self.build_pose(tool["toolbox_xyz"], tool["toolbox_rpy"])]
        co.object.operation = CollisionObject.ADD
        co.touch_links = ["pool", "tc_arm_side", "ur5e_wrist_3_link", "ur5e_wrist_2_link", "ur5e_wrist_1_link"]
        
        scene_msg2.robot_state.attached_collision_objects.append(co)
        self.pub_scene.publish(scene_msg2)
        
        self.get_logger().info(f"Dropped [{tool_name}] into toolbox slot.")

    def attach_tool_to_robot(self, tool_name):
        tool = self.tools[tool_name]
        
        if tool["mesh_msg"] is None:
            self.get_logger().error(f"Cannot attach {tool_name}: Mesh was not successfully loaded!")
            return
            
        import time
            
        # --- MESSAGE 1: Detach everything from everywhere and kill ghosts ---
        scene_msg1 = PlanningScene()
        scene_msg1.is_diff = True
        scene_msg1.robot_state.is_diff = True
        
        # Detach all other tools from the robot tip
        for name, other_tool in self.tools.items():
            if name != tool_name and other_tool["mesh_msg"] is not None:
                detach_obj = AttachedCollisionObject()
                detach_obj.object.id = other_tool["id"] + "_attached"
                detach_obj.link_name = "tc_arm_side"
                detach_obj.object.operation = CollisionObject.REMOVE
                scene_msg1.robot_state.attached_collision_objects.append(detach_obj)
                
                ghost_killer = CollisionObject()
                ghost_killer.id = other_tool["id"] + "_attached"
                ghost_killer.operation = CollisionObject.REMOVE
                scene_msg1.world.collision_objects.append(ghost_killer)
                
                # Drop them back into the toolbox
                co_drop = AttachedCollisionObject()
                co_drop.link_name = "pool"
                co_drop.object.id = other_tool["id"] + "_toolbox"
                co_drop.object.header.frame_id = "ur5e_base_link"
                co_drop.object.meshes = [other_tool["mesh_msg"]]
                co_drop.object.mesh_poses = [self.build_pose(other_tool["toolbox_xyz"], other_tool["toolbox_rpy"])]
                co_drop.object.operation = CollisionObject.ADD
                co_drop.touch_links = ["pool", "tc_arm_side", "ur5e_wrist_3_link", "ur5e_wrist_2_link", "ur5e_wrist_1_link"]
                scene_msg1.robot_state.attached_collision_objects.append(co_drop)

        # Remove the target tool from the toolbox
        detach_pool = AttachedCollisionObject()
        detach_pool.link_name = "pool"
        detach_pool.object.id = tool["id"] + "_toolbox"
        detach_pool.object.operation = CollisionObject.REMOVE
        scene_msg1.robot_state.attached_collision_objects.append(detach_pool)
        
        ghost_killer_target = CollisionObject()
        ghost_killer_target.id = tool["id"] + "_toolbox"
        ghost_killer_target.operation = CollisionObject.REMOVE
        scene_msg1.world.collision_objects.append(ghost_killer_target)
        
        self.pub_scene.publish(scene_msg1)
        
        # Give MoveIt 100ms to process the detachments
        time.sleep(0.1)
        
        # --- MESSAGE 2: Attach the target tool to the robot tip ---
        scene_msg2 = PlanningScene()
        scene_msg2.is_diff = True
        scene_msg2.robot_state.is_diff = True
        
        aco = AttachedCollisionObject()
        aco.link_name = "tc_arm_side"
        aco.object.id = tool["id"] + "_attached"
        aco.object.header.frame_id = "tc_arm_side"
        aco.object.meshes = [tool["mesh_msg"]]
        aco.object.mesh_poses = [self.build_pose(tool["attach_xyz"], tool["attach_rpy"])]
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ["tc_arm_side", "ur5e_wrist_3_link"]
        
        scene_msg2.robot_state.attached_collision_objects.append(aco)
        self.pub_scene.publish(scene_msg2)
        
        self.get_logger().info(f"Attached [{tool_name}] to the robot tip! (Auto-detached everything else)")

def main(args=None):
    rclpy.init(args=args)
    node = ToolManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
