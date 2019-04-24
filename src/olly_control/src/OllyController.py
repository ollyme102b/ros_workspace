#!/usr/bin/env python

import numpy as np
import time
import rospy
from geometry_msgs.msg import Pose, Twist

import imp

LaunchHelper = imp.load_source('LaunchHelper',
                               '/home/jasonanderson/ME102B_Project/ros_workspace/src/LaunchHelper.py')
from LaunchHelper import get_and_set_params


class OllyController(object):

    def __init__(self):
        rospy.init_node('orphaned_olly_controller')

        self._controller_name = rospy.get_name()
        self._olly_name = self._controller_name[0:6]

        params_dict = dict(step_time=0.1, object_length=1, horizon=10)
        params = get_and_set_params(self._controller_name, params_dict)

        self._step_time = float(params['step_time'])
        self._object_length = float(params['step_time'])
        self._horizon = int(params['horizon'])

        self._last_actuation_time = time.time()

        self._position = np.zeros((2,))
        self._position_subscriber = rospy.Subscriber(self._olly_name + "/position", Pose, self._position_callback)

        self._command_cache = Twist()
        self._command_publisher = rospy.Publisher(self._olly_name + "/velocity_command", Twist, queue_size=1)

    def _position_callback(self, message):
        self._position = np.array([message.x, message.y])

    def _publish_command(self):
        self._command_publisher.publish(self._command_cache)
        self._last_actuation_time = time.time()

    def _assert_control(self):
        if (time.time() - self._last_actuation_time) > self._step_time:
            optimal_xy_velocity = self._compute_control_action()
            self._command_cache.linear.x = optimal_xy_velocity[0]
            self._command_cache.linear.y = optimal_xy_velocity[1]
            self._publish_command()

    def _compute_control_action(self):
        raise RuntimeError(
            'Calling _compute_control_action() of abstract OllyController. OllyController._compute_control_action() must have an override')

    def effectuate_control(self):
        try:
            rate = rospy.Rate(1 / self._step_time)

            while not rospy.is_shutdown():
                print("Main Loop executing!")
                self._assert_control()
                rate.sleep()

        except rospy.ROSInterruptException:
            pass
