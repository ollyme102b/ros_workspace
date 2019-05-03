#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion

def run():
	rospy.init_node('joy_to_input')
	j = Joystick()

	rospy.Subscriber('/joy', Joy, j.joy_callback)
	rospy.Subscriber('/position', PoseStamped, j.position_callback)
	
	cmv_pub = rospy.Publisher('/velocity_cmd', Twist, queue_size=10)
	global_velocity_pub = rospy.Publisher('/global_velocity_cmd', Twist, queue_size=10)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		cmv_pub.publish(j.get_twist())
		global_velocity_pub.publish(j.get_inerital_twist())
		rate.sleep()
	
class Joystick:
	def __init__(self, max_vel=0.05, max_ang_vel=0.3):
		self.yaw = 0	# current orientation
		self.vx = 0		# linear x velocity body frame
		self.vy = 0		# linear y velocity body frame
		self.vxi = 0 	# inertial vx
		self.vyi = 0	# inertial vy
		self.wz = 0		# angular z velocity
		self.max_vel = max_vel 	# max linear velocity [m/s]
		self.max_ang_vel = max_ang_vel # max angular velocity [rad/s]

	def joy_callback(self, msg):
		a = 1 			# sensitivity: 0 = cubic, 1 = linear
		# vxi = self.max_vel*(-(a*pow(msg.axes[0],3) + (1-a)*msg.axes[0]))
		# vyi = self.max_vel*(a*pow(msg.axes[1],3) + (1-a)*msg.axes[1])
		vxi = -self.max_vel*np.sign(msg.axes[0])
		vyi = self.max_vel*np.sign(msg.axes[1])

		vxb, vyb = self.inertial_to_body(vxi, vyi)
		self.vxi = vxi
		self.vyi = vyi
		self.vx = vxb
		self.vy = vyb
		# self.wz = self.max_vel*(a*pow(msg.axes[3],3) + (1-a)*msg.axes[3])
		self.wz = self.max_ang_vel*np.sign(msg.axes[3])

	def position_callback(self, msg):
		quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
		self.yaw = euler_from_quaternion(quaternion)[2]

	def get_twist(self):
		t = Twist()
		t.linear.x = self.vx
		t.linear.y = self.vy
		t.angular.z = self.wz
		return t

	def get_inerital_twist(self):
		t = Twist()
		t.linear.x = self.vxi
		t.linear.y = self.vyi
		t.angular.z = self.wz
		return t


	def inertial_to_body(self, xi, yi):
		xb = np.cos(self.yaw) * xi + np.sin(self.yaw) * yi
		yb = -np.sin(self.yaw) * xi + np.cos(self.yaw) * yi
		return xb, yb

if __name__ == '__main__':
    try:
		run()
    except rospy.ROSInterruptException:
		pass