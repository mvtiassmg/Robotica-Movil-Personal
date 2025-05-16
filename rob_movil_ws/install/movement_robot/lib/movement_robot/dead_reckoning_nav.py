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

        self.max_v = 0.2  # m/s
        self.max_w = 1.0  # rad/s
        self.factor_rotacion = 1.10
        # Para ajustar el tiempo de rotación

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
        self.pose_stamped_sub = self.create_subscription(Pose, '/real_pose', self.real_pose_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.ocupancy_sub = self.create_subscription(Vector3, '/occupancy_state', self.occupancy_cb, 10)

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

            while self.obstaculo:
                self.get_logger().warn('Obstáculo detectado. Esperando...')
                time.sleep(0.5)

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

        # Normalizar ángulo entre -pi y pi
        delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))

        self.get_logger().info(f'Ángulo deseado: {angulo_deseado:.2f}, Delta theta: {delta_theta:.2f}')

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

        # Actualizar posición estimada
        self.actual_position.poses[0].position.x = x
        self.actual_position.poses[0].position.y = y
        self.actual_position.poses[0].orientation.z = theta

    def accion_mover_cb(self, pose_list):
        self.get_logger().info('Recibida nueva lista de objetivos')
        for pose in pose_list.poses:
            self.mover_robot_a_destino(pose)
        self.guardar_trayectoria()

    def real_pose_cb(self, msg_pos):
        self.real_trayectoria.append((msg_pos.position.x, msg_pos.position.y))
        self.get_logger().info(f'Posición real - X: {msg_pos.position.x}. Y: {msg_pos.position.y}')

    def odom_cb(self, msg):
        self.odom_trayectoria.append((
            msg.pose.pose.position.x,
            msg.pose.pose.position.y))
        self.get_logger().info(f'Posición odometria - X: {msg.pose.pose.position.x}. Y: {msg.pose.pose.position.y}')

    def guardar_trayectoria(self):
        with open('trayectoria_real.txt', 'w') as f_real, open('trayectoria_odom.txt', 'w') as f_odom:
            for x, y in self.real_trayectoria:
                f_real.write(f'{x},{y}\n')
            for x, y in self.odom_trayectoria:
                f_odom.write(f'{x},{y}\n')

        self.get_logger().info('Trayectorias guardadas en "trayectoria_real.txt" y "trayectoria_odom.txt".')

    def occupancy_cb(self, msg_oc):
        state_vector = [msg_oc.x, msg_oc.y, msg_oc.z]
        posiciones = []

        if state_vector == [0.0, 0.0, 0.0]:
            self.obstaculo = False
            self.get_logger().info('Camino despejado')
        else:
            self.obstaculo = True
            if state_vector[0] == 1.0:
                posiciones.append('Obstáculo a la izquierda')
            if state_vector[1] == 1.0:
                posiciones.append('Obstáculo al frente')
            if state_vector[2] == 1.0:
                posiciones.append('Obstáculo a la derecha')

            self.get_logger().warn(' / '.join(posiciones))

def main():
    rclpy.init()
    node = DeadReckoningNav()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
