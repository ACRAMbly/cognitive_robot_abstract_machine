import geometry_msgs.msg
import rclpy
from perception_interfaces.srv import GetObjPoses

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import HomogeneousTransformationMatrix as TransformationMatrix
from semantic_digital_twin.world import World

prefix = "PhysicalObject"


def update_obj_position(poses: list[tuple[str, geometry_msgs.msg.PoseStamped]], world: World):
    for pose in poses:
        obj_name = pose[0]
        obj_pose = pose[1]
        obj_trans = TransformationMatrix.from_xyz_quaternion(obj_pose.pose.position.x, obj_pose.pose.position.y,
                                                             obj_pose.pose.position.z, obj_pose.pose.orientation.x,
                                                             obj_pose.pose.orientation.y, obj_pose.pose.orientation.z,
                                                             obj_pose.pose.orientation.w,
                                                             world.get_kinematic_structure_entity_by_name(
                                                                 PrefixedName(obj_pose.header.frame_id, "tracy")))

        with world.modify_world():
            transform = world.transform(
                obj_trans, world.get_body_by_name(PrefixedName(obj_name, prefix)).parent_connection.parent)
            connection = world.get_body_by_name(PrefixedName(obj_name, prefix)).parent_connection
            connection.origin = transform


class PerceptionClientSingle:
    def __init__(self, world, node):
        self.world = world
        self.node = node
        self.client = self.node.create_client(GetObjPoses, "cube_poses")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.req = GetObjPoses.Request()
        self.transform = None

    def request(self, obj):
        self.req.object = obj
        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        pose = [(obj, response.pose)]
        update_obj_position(pose, self.world)
