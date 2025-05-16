#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty
import sympy as sp
import numpy as np

class PIDController( Node ):
  def __init__( self, kp, ki, kd ):
    super().__init__( 'p_controller' )
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.setpoint = None
    self.state = None
    self.proportional_action = 0

    self.actuation_pub = self.create_publisher( Float64, 'control_effort', 1 )
    self.dist_set_point_sub = self.create_subscription( Float64, 'setpoint', self.setpoint_cb, 1 )
    self.dist_state_sub = self.create_subscription( Float64, 'state', self.state_cb, 1 )

    self.error_anterior = None
    self.error_acumulado = 0
    self.tiempo_anterior = self.get_clock().now()

  def setpoint_cb( self, msg ):
    self.get_logger().info( '[PICTRL] new setpoint received: %.2f' % (msg.data) )
    self.reset()
    self.setpoint = msg.data

  def state_cb( self, msg ):
    if self.setpoint is None:
        return

    self.state = msg.data
    error = self.setpoint - self.state

    # Tiempo 
    tiempo_actual = self.get_clock().now()
    Ts = (tiempo_actual - self.tiempo_anterior).nanoseconds * 1e-9
    self.tiempo_anterior = tiempo_actual  # Actualiza para la siguiente iteración

    # Proporcional
    p_actuation = self.kp * error

    # Integrativo
    self.error_acumulado += error
    i_actuation = self.ki * Ts * self.error_acumulado 

    # Derivativo
    if self.error_anterior is not None and Ts > 0:
        d_actuation = self.kd * (error - self.error_anterior) / Ts
    else:
        d_actuation = 0.0

    # Guardar error actual
    self.error_anterior = error

    # Actuación
    actuation = p_actuation + i_actuation + d_actuation

    msg = Float64()
    msg.data = actuation
    self.actuation_pub.publish( msg )

  def reset( self ):
    self.setpoint = None
    self.state = None

    self.last_error = None
    self.error_acumulado = 0

def main():
  rclpy.init()
  p_ctrl = PIDController( 0.5, 0, 0 )
  rclpy.spin( p_ctrl )

if __name__ == '__main__':
  main()