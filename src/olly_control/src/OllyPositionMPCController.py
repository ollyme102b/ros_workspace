#!/usr/bin/env python

import numpy as np
import rospy
import imp

PositionMPCController2D = imp.load_source('PositionMPCController2D',
                                          '/home/jasonanderson/ME102B_Project/ros_workspace/src/olly_control/src/v0/PositionMPCController2D.py')

from PositionMPCController2D import PositionMPCController2D
from geometry_msgs.msg import Pose, Twist

from OllyController import OllyController


class OllyPositionMPCController(OllyController):
    def __init__(self):
        super(OllyPositionMPCController, self).__init__()

        self._position_set_point = np.zeros((2,))
        self._position_set_point_subscriber = rospy.Subscriber(self._olly_name + "/position_set_point",
                                                               Pose,
                                                               self._position_set_point_callback)

        self._controller = PositionMPCController2D(step_time=self._step_time, horizon=self._horizon)

    def _position_set_point_callback(self, message):
        self._position_set_point = np.array([message.x, message.y])

    def _compute_control_action(self):
        return self._controller.update_then_calculate_optimal_actuation(self._position, self._position_set_point)


if __name__ == "__main__":
    controller_node = OllyPositionMPCController()
    controller_node.effectuate_control()
