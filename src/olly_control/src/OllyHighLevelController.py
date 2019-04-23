#!/usr/bin/env python

import numpy as np
import time
import rospy
import imp

FollyHighLevelControllerV2 = imp.load_source('FollyHighLevelControllerV2',
                                             '/home/jasonanderson/ME102B_Project/ros_workspace/src/olly_control/src/v2/FollyHighLevelControllerV2.py')
LaunchHelper = imp.load_source('LaunchHelper',
                               '/home/jasonanderson/ME102B_Project/ros_workspace/src/LaunchHelper.py')
from FollyHighLevelControllerV2 import FollyHighLevelControllerV2
from LaunchHelper import get_and_set_params

# useful ROS message types
from geometry_msgs.msg import Pose, Twist, Wrench


class OllyHighLevelController:

    def __init__(self):

        self._high_level_controller_name = rospy.get_name()
        self._olly_name = self._high_level_controller_name[0:6]

        params_dict = dict(step_time=None)
        params = get_and_set_params(self._high_level_controller_name, params_dict)

        self._step_time = float(params['step_time'])
        self._last_actuation_time = time.time()
        self._controller = FollyHighLevelControllerV2(step_time=self._step_time)

        self._command_cache = Twist()
        self._position = np.zeros((2,))
        self._companion_position = np.zeros((2,))
        self._companion_velocity = np.zeros((2,))

        self._control_publisher = rospy.Publisher(self._olly_name + "/velocity_command",
                                                  Twist, queue_size=1)
        if self._olly_name == 'folly':
            self._companion_name = 'molly'
        else:
            self._companion_name = 'folly'

        self._companion_position_subscriber = rospy.Subscriber(self._companion_name + "/position",
                                                               Pose,
                                                               OllyHighLevelController.companion_position_callback)

        self._companion_velocity_subscriber = rospy.Subscriber(self._companion_name + "/velocity",
                                                               Twist,
                                                               OllyHighLevelController.companion_velocity_callback)

    def position_callback(self, message):
        self._position = np.array([message.x, message.y])

    def companion_position_callback(self, message):
        self._companion_position = np.array([message.x, message.y])

    def companion_velocity_callback(self, message):
        self._companion_velocity = np.array([message.x, message.y])

    def assert_control(self):
        if (time.time() - self._last_actuation_time) > self._step_time:
            optimal_xy_velocity = self._controller.update_then_calculate_optimal_actuation(self._companion_position,
                                                                                           self._position,
                                                                                           self._companion_velocity)
            self._command_cache.linear.x = optimal_xy_velocity[0]
            self._command_cache.linear.y = optimal_xy_velocity[1]
            self._send_command()

    def _send_command(self):
        self._control_publisher.publish(self._command_cache)
        self._last_actuation_time = time.time()


if __name__ == "__main__":
    try:
        rospy.init_node('orphaned_olly_high_level_controller')

        olly_high_level_controller = OllyHighLevelController()

        rate = rospy.Rate(1 / olly_high_level_controller._step_time)

        while not rospy.is_shutdown():
            olly_high_level_controller.assert_control()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
