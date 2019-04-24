#!/usr/bin/env python

import numpy as np
import time
import rospy
import imp

FollyHighLevelControllerV1 = imp.load_source('FollyHighLevelControllerV1',
                                             '/home/jasonanderson/ME102B_Project/ros_workspace/src/olly_control/src/v1/FollyHighLevelControllerV1.py')
FollyHighLevelControllerV2 = imp.load_source('FollyHighLevelControllerV2',
                                             '/home/jasonanderson/ME102B_Project/ros_workspace/src/olly_control/src/v2/FollyHighLevelControllerV2.py')
from FollyHighLevelControllerV1 import FollyHighLevelControllerV1
from FollyHighLevelControllerV2 import FollyHighLevelControllerV2

# useful ROS message types
from geometry_msgs.msg import Pose, Twist, Wrench

from OllyController import OllyController


class OllyHighLevelControllerV2(OllyController):

    def __init__(self):
        super(OllyHighLevelControllerV2, self).__init__()

        self._companion_name = 'molly'

        self._companion_position = np.zeros((2,))
        self._companion_position_subscriber = rospy.Subscriber(self._companion_name + "/position",
                                                               Pose,
                                                               self.companion_position_callback)

        self._companion_velocity = np.zeros((2,))
        self._companion_velocity_subscriber = rospy.Subscriber(self._companion_name + "/velocity",
                                                               Twist,
                                                               self.companion_velocity_callback)

        self._controller = FollyHighLevelControllerV2(step_time=self._step_time, object_length = self._object_length)

    def companion_position_callback(self, message):
        self._companion_position = np.array([message.x, message.y])

    def companion_velocity_callback(self, message):
        self._companion_velocity = np.array([message.x, message.y])

    def _assert_control(self):
        if (time.time() - self._last_actuation_time) > self._step_time:
            optimal_xy_velocity = self._controller.update_then_calculate_optimal_actuation(self._companion_position,
                                                                                           self._position,
                                                                                           self._companion_velocity)
            self._command_cache.linear.x = optimal_xy_velocity[0]
            self._command_cache.linear.y = optimal_xy_velocity[1]
            self._publish_command()


if __name__ == "__main__":
    controller_node = OllyHighLevelControllerV2()
    controller_node.effectuate_control()
