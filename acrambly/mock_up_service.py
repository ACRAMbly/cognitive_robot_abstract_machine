import geometry_msgs.msg
from perception_interfaces.srv import GetObjPoses
import rclpy.node as node

from semantic_digital_twin.spatial_types.spatial_types import HomogeneousTransformationMatrix as tm

class GetCubePosesService(node.Node):

    def __init__(self):
        super().__init__('perception')
        self.srv = self.create_service(GetObjPoses, 'cube_poses', self.callback)

    def callback(self, request, response):
        response.pose = detect_pose(request.object)
        return response

def detect_pose(obj):
    print(obj)
    if obj == "red_box":
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "camera_color_optical_frame"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 0.051, -0.149, 0.987
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = 0.701, -0.695, 0.104, 0.116
        print(pose)
        return pose
    elif obj == "yellow_box":
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "camera_color_optical_frame"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = -0.449, -0.151, 0.996
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = 0.701, -0.695, 0.104, 0.116
        print(pose)
        return pose
    elif obj == "blue_box":
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "camera_color_optical_frame"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = -0.199, -0.15, 0.991
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = 0.701, -0.695, 0.104, 0.116
        print(pose)
        return pose


import rclpy
rclpy.init()

service = GetCubePosesService()
rclpy.spin(service)
rclpy.shutdown()