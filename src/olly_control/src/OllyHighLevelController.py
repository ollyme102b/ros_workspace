#!/usr/bin/env python

import numpy as np
import time
import rospy
from src.olly_control.src.v1.FollyHighLevelControllerV1 import FollyHighLevelControllerV1
from src.olly_control.src.v2.FollyHighLevelControllerV2 import FollyHighLevelControllerV2

# useful ROS message types
from geometry_msgs.msg import Pose, Twist, Wrench
from src.LaunchHelper import get_and_set_params


def main():
    rospy.init_node('orphanned_olly_high_level_controller')

    ollyHighLevelController = OllyHighLevelController()

    while not rospy.is_shutdown():
        ollyHighLevelController.assert_control()
        rate.sleep()


class OllyHighLevelController():

    def __init__(self, ):

        self._high_level_controller_name = rospy.get_name()
        self._olly_name = self._high_level_controller_name[0:5]

        params_dict = dict(step_time=None)
        params = get_and_set_params(self._olly_name, params_dict)

        self._step_time = params['step_time']
        self._last_actuation_time = time.time()
        self._controller = FollyHighLevelControllerV2(step_time=self._step_time)

        self._command_cache = Pose()
        self._position = np.zeros((3,))
        self._companion_position = np.zeros((3,))
        self._companion_velocity = np.zeros((3,))

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
        self._position = np.array([message.x, message.y, message.z])

    def companion_position_callback(self, message):
        self._companion_position = np.array([message.x, message.y, message.z])

    def companion_velocity_callback(self, message):
        self._companion_velocity = np.array([message.x, message.y, message.z])

    def assert_control(self):
        if (time.time() - self._last_actuation_time) > self._step_time:
            optimal_xy_velocity = self._controller.update_then_calculate_optimal_actuation(self._companion_position,
                                                                                           self._position,
                                                                                           self._companion_velocity)
            self._command_cache.x = optimal_xy_velocity[0]
            self._command_cache.y = optimal_xy_velocity[1]
            self._send_command()

    def _send_command(self):
        self._control_publisher.publish(self._command_cache)
        self._last_actuation_time = time.time()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
