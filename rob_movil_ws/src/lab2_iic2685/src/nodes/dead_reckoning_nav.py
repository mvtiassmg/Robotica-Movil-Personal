#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseArray, Vector3
from nav_msgs.msg import Odometry
import math

# Dead reckoning:
# v = d/t => t = d/v => d = v * t
# w = θ/t => t = θ/w => θ = w * t

class DeadReckoningNav(Node):
    def __init__(self):
        super().__init__('dead_reckoning_nav')

        # Parámetros de velocidad
        self.max_v = 0.2  # m/s
        self.max_w = 1.0  # rad/s
        self.factor_rotacion = 1.0  # Para ajustar el tiempo de rotación

        # Posición inicial del robot
        self.actual_position = PoseArray()
        self.actual_position.poses = [Pose()]
        self.actual_position.poses[0].position.x = 1.0
        self.actual_position.poses[0].position.y = 1.0
        self.actual_position.poses[0].orientation.z = 0.0

        # Publicadores y suscriptores
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_mux/cmd_vel', 10)
        self.commands_velocity = self.create_publisher(Twist, 'commands/velocity', 10)
        self.pose_arr_sub = self.create_subscription(PoseArray, '/goal_list', self.accion_mover_cb, 10)

        # Estado del sistema
        self.obstaculo = False
        self.real_trayectoria = []
        self.odom_trayectoria = []

        self.get_logger().info('Nodo dead_reckoning_nav inicializado correctamente.')

    def aplicar_velocidad(self, speed_command_list):
        for v, w, t in speed_command_list:
            msg_vel = Twist()

            if w != 0:
                t *= self.factor_rotacion

            msg_vel.linear.x = float(v)
            msg_vel.angular.z = float(w)
            self.cmd_vel_pub.publish(msg_vel)
            self.get_logger().info(f'Comando: v = {v}, w = {w}, t = {t:.2f}')
            time.sleep(t)

        # Detener robot al final
        msg_vel.linear.x = 0.0
        msg_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(msg_vel)
        self.get_logger().info('Movimiento finalizado')
        time.sleep(1)

    def mover_robot_a_destino(self, goal_pose):
        x, y = goal_pose.position.x, goal_pose.position.y
        theta = goal_pose.orientation.z

        x0 = self.actual_position.poses[0].position.x
        y0 = self.actual_position.poses[0].position.y
        theta_0 = self.actual_position.poses[0].orientation.z

        delta_x = x - x0
        delta_y = y - y0
        angulo_deseado = math.atan2(delta_y, delta_x)
        delta_theta = angulo_deseado - theta_0

        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
        t_rot1 = abs(delta_theta) / self.max_w
        w_rot1 = self.max_w if delta_theta >= 0.0 else -self.max_w
        rot_inicial = (0.0, w_rot1, t_rot1)

        distancia = math.sqrt(delta_x**2 + delta_y**2)
        t_avance = distancia / self.max_v
        mov_lineal = (self.max_v, 0.0, t_avance)

        delta_theta2 = theta - angulo_deseado
        delta_theta2 = math.atan2(math.sin(delta_theta2), math.cos(delta_theta2))

        t_rot2 = abs(delta_theta2) / self.max_w
        w_rot2 = self.max_w if delta_theta2 >= 0.0 else -self.max_w
        rot_final = (0.0, w_rot2, t_rot2)

        speed_command_list = [rot_inicial, mov_lineal, rot_final]
        self.aplicar_velocidad(speed_command_list)

        self.actual_position.poses[0].position.x = x
        self.actual_position.poses[0].position.y = y
        self.actual_position.poses[0].orientation.z = theta

    def accion_mover_cb(self, pose_list):
        self.get_logger().info('Recibida nueva lista de objetivos')
        for pose in pose_list.poses:
            self.mover_robot_a_destino(pose)


def main():
    rclpy.init()
    node = DeadReckoningNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()