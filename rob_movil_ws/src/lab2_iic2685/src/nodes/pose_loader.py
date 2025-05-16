#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray

class PoseLoader(Node):
    def __init__(self):
        super().__init__('pose_loader')
        self.publisher = self.create_publisher(PoseArray, '/goal_list', 10)
        self.timer = self.create_timer(1.0, self.publicar_poses) 
        self.publicado = False

    def publicar_poses(self):
        if self.publicado:
            return

        poses = PoseArray()
        poses.header.frame_id = 'map'

        # Lista embebida con orientación optimizada (x, y, theta)
        coordenadas = [
            (2.0, 1.0, 1.57),
            (2.0, 2.0, 3.14),
            (1.0, 2.0, -1.57),
            (1.0, 1.0, 0.0),
            (2.0, 1.0, 1.57),
            (2.0, 2.0, 3.14),
            (1.0, 2.0, -1.57),
            (1.0, 1.0, 0.0),
            (2.0, 1.0, 1.57),
            (2.0, 2.0, 3.14),
            (1.0, 2.0, -1.57),
            (1.0, 1.0, 0.0),
        ]

        for x, y, theta in coordenadas:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y

            pose.orientation.z = theta

            poses.poses.append(pose)

        self.publisher.publish(poses)
        self.get_logger().info(f'Publicadas {len(poses.poses)} poses al tópico /goal_list')

        self.publicado = True
        self.get_logger().info('Nodo pose_loader se cerrará luego de publicar.')
        self.destroy_node()

def main():
    rclpy.init()
    node = PoseLoader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
