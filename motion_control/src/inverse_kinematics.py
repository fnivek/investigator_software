#!/usr/bin/env python

"""
The primative driver converts desired wheel states (position, velocity, ...) to pwm 
signals for the MSP430 to set 
"""
import rospy
from geometry_msgs.msg import Twist
from motion_control.msg import MotorAngularWheelVelocities
from hardware_interface.msg import EncoderSpeed

class InverseKinematics:
    def __init__(self):
      rospy.init_node('inverse_kinematics', anonymous = False)

      # Grab params
      self.wheel_base = rospy.get_param('/wheel_base', 0.3048) # 0.3048 m = 12 in
      self.wheel_radius = rospy.get_param('/wheel_radius', 0.06)
      self.simulate = rospy.get_param('~simulate', False)

      # Publishers
      self.wheel_ang_vel_pub = rospy.Publisher('motor_angular_wheel_velocities', MotorAngularWheelVelocities, queue_size = 1)
      self.simulation_pub = rospy.Publisher('encoder_speed', EncoderSpeed, queue_size = 1)

      # Subscriber
      self.twist_sub = rospy.Subscriber('twist', Twist, self.twist_cb, queue_size = 1)

    """
    This function converts the desired state and maps it to the wheels

    Variables:
      wr := right wheel angular velocity (rad/s)
      wl := left wheel angular velocity (rad/s)
      r_wheel := radius of the wheel (m)
      Vr := velocity of right wheel along the ground (m/s)
      Vl := velocity of left wheel along the ground (m/s)
      Vx := Linear velocity of robot (m/s)
      w := angular velocity of robot about instantaneous center of curvature (ICC) (rad/s)
      L := length between the wheels (m)
    
    Equation:
      Vr = wr * r_wheel
      Vl = wl * r_wheel
      Vx = (Vr + Vl) / 2
      w = (Vr - Vl) / L
    ------------------------
      Vr = Vx + L * w / 2
      Vl = Vx - L * w / 2
    ------------------------
      wr = (1 / r_wheel) * (Vx + L * w / 2)
      wl = (1 / r_wheel) * (Vx - L * w / 2)

    """
    def twist_cb(self, twist):
      Vx = twist.linear.x
      w = twist.angular.z

      left_wheel = (Vx - self.wheel_base * w / 2) / self.wheel_radius
      right_wheel = (Vx + self.wheel_base * w / 2) / self.wheel_radius

      self.wheel_ang_vel_pub.publish(
        MotorAngularWheelVelocities(
          left_angular_velocity = left_wheel, 
          right_angular_velocity = right_wheel))

      if self.simulate:
        self.sim(left_wheel, right_wheel)

    def sim(self, left, right):
      self.simulation_pub.publish(
        EncoderSpeed(
          left_motor = left,
          right_motor = right))


if __name__ == '__main__':
  ik = InverseKinematics()
  rospy.spin()