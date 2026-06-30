# Scratch script to verify coordinates
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

class FKTest(Node):
    def __init__(self):
        super().__init__('fk_test')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        
    def get_fk(self, joints):
        req = GetPositionFK.Request()
        req.header.frame_id = 'ur5e_base_link'
        req.fk_link_names = ['ur5e_tool0']
        req.robot_state.joint_state.name = [
            'ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
            'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint'
        ]
        req.robot_state.joint_state.position = joints
        self.fk_client.wait_for_service()
        future = self.fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().pose_stamped[0].pose

def main():
    rclpy.init()
    node = FKTest()
    take = [-2.386735264454977, -2.239531179467672, -1.259566068649292, -1.213236854677536, 1.5708116292953491, 3.110840082168579]
    drop = [-2.402625624333517, -1.7159730396666468, -2.0344910621643066, -0.9617884916118165, 1.570833683013916, 3.0950727462768555]
    
    pose_take = node.get_fk(take)
    pose_drop = node.get_fk(drop)
    print(f"TAKE: x={pose_take.position.x}, y={pose_take.position.y}, z={pose_take.position.z}")
    print(f"DROP: x={pose_drop.position.x}, y={pose_drop.position.y}, z={pose_drop.position.z}")
    
    # Calculate difference
    dx = pose_drop.position.x - pose_take.position.x
    dy = pose_drop.position.y - pose_take.position.y
    dz = pose_drop.position.z - pose_take.position.z
    print(f"DIFF: dx={dx}, dy={dy}, dz={dz}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
