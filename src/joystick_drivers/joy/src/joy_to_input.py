#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def run():
	rospy.init_node('joy_to_input')
	j = Joystick()
	rospy.Subscriber('/joy', Joy, j.callback)
	cmv_pub = rospy.Publisher('/velocity_cmd', Twist, queue_size=10)

	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		cmv_pub.publish(j.get_twist())
		rate.sleep()
	
class Joystick:
	def __init__(self):
		self.vx = 0		# linear x velocity
		self.vy = 0		# linear y velocity
		self.wz = 0		# angular z velocity

	def callback(self, msg):
		a = 1
		self.vx = -(a*pow(msg.axes[0],3) + (1-a)*msg.axes[0])
		self.vy = a*pow(msg.axes[1],3) + (1-a)*msg.axes[1]
		self.wz = a*pow(msg.axes[3],3) + (1-a)*msg.axes[3]

	def get_twist(self):
		t = Twist()
		t.linear.x = self.vx
		t.linear.y = self.vy
		t.angular.z = self.wz 
		return t

if __name__ == '__main__':
    try:
		run()
    except rospy.ROSInterruptException:
		pass
