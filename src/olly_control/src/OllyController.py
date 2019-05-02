#!/usr/bin/env python

import numpy as np
import time
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
import sys
import os

sys.path.append(sys.path[0].replace('/olly_control/src',''))
from LaunchHelper import get_and_set_params


class OllyController(object):
    """
    Abstract Olly Controller ROS Node class
    Includes all class attributes and methods that pertain to all olly controller ROS nodes
    """

    def __init__(self):
        rospy.init_node('orphaned_olly_controller')  # initialize ROS node
        rospy.on_shutdown(self._shutdown)

        self._controller_name = rospy.get_name()

        # grab params from launch file
        params_dict = dict(step_time=0.1, object_length=1, horizon=10, olly_name=None, companion_name='no_companion')
        params = get_and_set_params(self._controller_name, params_dict)
        # set params as atributtes
        self._olly_name = str(params['olly_name'])
        self._companion_name = str(params['companion_name'])
        assert self._olly_name is not None, 'olly_name from launch file cannot be None'
        self._step_time = float(params['step_time'])
        self._object_length = float(params['step_time'])
        self._horizon = int(params['horizon'])

        self._last_actuation_time = time.time()  # set first actuation time

        # initialize position array and create ROS subscriber
        self._position = np.zeros((3,))
        self._position_subscriber = rospy.Subscriber('/' + self._olly_name + "/position", PoseStamped, self._position_callback)

        # initialize command cache and create ROS publisher
        self._command_cache = Twist()
        self._command_publisher = rospy.Publisher('/' + self._olly_name + "/velocity_cmd", Twist, queue_size=1)

    def _position_callback(self, message):
        """
        position ROS call back function
        :param message: of ROS message type Pose
        """
        orientation = message.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        yaw = euler_from_quaternion(quaternion)[2]
        self._position = np.array([message.pose.position.x, message.pose.position.y, yaw])


    def _publish_command(self):
        """
        publish command cache and reset last actuation time
        """
        min_actuation = 0.001
        rounded_command = Twist()
        rounded_command.linear.x = self._command_cache.linear.x if abs(self._command_cache.linear.x) > min_actuation else 0.0
        rounded_command.linear.y = self._command_cache.linear.y if abs(self._command_cache.linear.y) > min_actuation else 0.0
        rounded_command.angular.z = self._command_cache.angular.z if abs(self._command_cache.angular.z) > min_actuation else 0.0 
        self._command_publisher.publish(rounded_command)
        self._last_actuation_time = time.time()

    def _assert_control(self):
        """
        compute and publish command after enough time has ellasped since the last actuation time
        """
        if (time.time() - self._last_actuation_time) >= self._step_time:
            optimal_xy_velocity = self._compute_control_action()
            xb, yb = self._inertial_to_body(optimal_xy_velocity[0], optimal_xy_velocity[1])
            self._command_cache.linear.x = xb
            self._command_cache.linear.y = yb
            self._command_cache.angular.z = optimal_xy_velocity[2]
            self._publish_command()

    def _compute_control_action(self):
        """
        abstract, must be overwritten by subclass
        """
        raise RuntimeError(
            'Calling _compute_control_action() of abstract OllyController. OllyController._compute_control_action() must have an override')

    def _inertial_to_body(self, xi, yi):
        yaw = self._position[2]
        xb =  np.cos(yaw) * xi + np.sin(yaw) * yi
        yb = -np.sin(yaw) * xi + np.cos(yaw) * yi

        return xb, yb

    def _shutdown(self):
        self._command_cache = Twist()
        self._publish_command()

    def effectuate_control(self):
        """
        main ROS loop
        """
        try:
            rate = rospy.Rate(1 / self._step_time)
            print("Main Loop executing for %s" % self._controller_name)
            while not rospy.is_shutdown():
                self._assert_control()
                rate.sleep()

        except rospy.ROSInterruptException:
            pass
