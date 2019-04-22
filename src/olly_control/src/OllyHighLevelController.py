#!/usr/bin/env python

import numpy as np
import rospy

# useful ROS message types
from geometry_msgs.msg import Pose, Twist, Wrench

class OllyHighLevelController:
	def __init__(self, name='folly'):

		rospy.init_node(name+'_high_level_controller')
	




