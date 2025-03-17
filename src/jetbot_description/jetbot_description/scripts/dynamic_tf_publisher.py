import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class DynamicTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 20, self.broadcast_transforms)

    def broadcast_transforms(self):
        # Kamera transform
        t_camera = TransformStamped()
        t_camera.header.stamp = self.get_clock().now().to_msg()
        t_camera.header.frame_id = 'base_link'
        t_camera.child_frame_id = 'camera'
        t_camera.transform.translation.x = 0.1
        t_camera.transform.translation.y = 0.0
        t_camera.transform.translation.z = 0.03
        t_camera.transform.rotation.x = 0.0
        t_camera.transform.rotation.y = 0.0
        t_camera.transform.rotation.z = 0.0
        t_camera.transform.rotation.w = 1.0
        self.br.sendTransform(t_camera)

        # Lidar_base transform
        t_lidar_base = TransformStamped()
        t_lidar_base.header.stamp = self.get_clock().now().to_msg()
        t_lidar_base.header.frame_id = 'base_link'
        t_lidar_base.child_frame_id = 'lidar_base'
        t_lidar_base.transform.translation.x = 0.0
        t_lidar_base.transform.translation.y = 0.0
        t_lidar_base.transform.translation.z = 0.015
        t_lidar_base.transform.rotation.x = 0.0
        t_lidar_base.transform.rotation.y = 0.0
        t_lidar_base.transform.rotation.z = 0.0
        t_lidar_base.transform.rotation.w = 1.0
        self.br.sendTransform(t_lidar_base)

        # Lidar transform
        t_lidar = TransformStamped()
        t_lidar.header.stamp = self.get_clock().now().to_msg()
        t_lidar.header.frame_id = 'lidar_base'
        t_lidar.child_frame_id = 'lidar'
        t_lidar.transform.translation.x = 0.0
        t_lidar.transform.translation.y = 0.0
        t_lidar.transform.translation.z = 0.035
        qz = math.sin(math.pi / 2)
        qw = math.cos(math.pi / 2)
        self.br.sendTransform(t_lidar)

def main():
    rclpy.init()
    node = DynamicTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
