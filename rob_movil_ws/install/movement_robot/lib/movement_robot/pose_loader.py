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

        try:
            with open('/home/maty/rob_movil_ws/src/movement_robot/movement_robot/poses.txt', 'r') as archivo:
                for linea in archivo:
                    x, y, theta = map(float, linea.strip().split(','))
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.orientation.z = theta
                    poses.poses.append(pose)
        except Exception as e:
            self.get_logger().error(f'Error leyendo el archivo de poses: {e}')
            return

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