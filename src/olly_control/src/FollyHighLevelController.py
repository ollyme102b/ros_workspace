#!/usr/bin/env python

import numpy as np
import rospy
import imp

ros_workspace_path = '/home/jasonanderson/ME102B_Project/ros_workspace'

FollyHighLevelControllerV1 = imp.load_source('FollyHighLevelControllerV1', ros_workspace_path+'/src/olly_control/src/v1/FollyHighLevelControllerV1.py')
FollyHighLevelControllerV2 = imp.load_source('FollyHighLevelControllerV2', ros_workspace_path+'/src/olly_control/src/v2/FollyHighLevelControllerV2.py')
from FollyHighLevelControllerV1 import FollyHighLevelControllerV1
from FollyHighLevelControllerV2 import FollyHighLevelControllerV2

# useful ROS message types
from geometry_msgs.msg import Pose, Twist

from OllyController import OllyController


class FollyHighLevelController(OllyController):
    """
    Folly ROS node for V1 and V2 path planners
    """

    def __init__(self):
        super(FollyHighLevelController, self).__init__()  # initialize parent class node

        # initialize companion position array and create ROS subscriber
        self._companion_position = np.zeros((2,))
        self._companion_position_subscriber = rospy.Subscriber('/' + self._companion_name + "/position",
                                                               Pose,
                                                               self._companion_position_callback)

        # initialize companion velocity array and create ROS subscriber
        self._companion_velocity = np.zeros((2,))
        self._companion_velocity_subscriber = rospy.Subscriber('/' + self._companion_name + "/velocity",
                                                               Twist,
                                                               self._companion_velocity_callback)

        # initialize controller object
        self._controller = FollyHighLevelControllerV1(step_time=self._step_time, object_length=self._object_length,
                                                      horizon=self._horizon)

    def _companion_position_callback(self, message):
        """
        companion position ROS call back function
        :param message: of ROS message type Pose
        """
        self._companion_position = np.array([message.position.x, message.position.y])

    def _companion_velocity_callback(self, message):
        """
        companion velocity ROS call back function
        :param message: of ROS message type Twist
        """
        self._companion_velocity = np.array([message.linear.x, message.linear.y])

    def _compute_control_action(self):
        """
        @override
        provides update to controller object and returns optimal control action
        :return:
        """
        return self._controller.update_then_calculate_optimal_actuation(self._companion_position,
                                                                        self._position,
                                                                        self._companion_velocity)


if __name__ == "__main__":
    controller_node = FollyHighLevelController()  # initialize node
    controller_node.effectuate_control()  # effectuate control
