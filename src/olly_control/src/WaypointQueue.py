#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
import sys

sys.path.append(sys.path[0].replace('/olly_control/src', ''))
from LaunchHelper import get_and_set_params


class WaypointQueue:
    def __init__(self):

        rospy.init_node('orphaned_waypoint_queue')
        self._waypoint_queue_name = rospy.get_name()

        params_dict = dict(step_time=1, olly_name=None, tolerance_radius=0.02)
        params = get_and_set_params(self._waypoint_queue_name, params_dict)

        self._olly_name = str(params['olly_name'])

        self._step_time = float(params['step_time'])
        self._tolerance_radius = float(params['tolerance_radius'])
        self.queue = []

        self._olly_setpoint = Pose()
        self._olly_setpoint_publisher = rospy.Publisher('/' + self._olly_name + "/position_setpoint", Pose,
                                                        queue_size=1)

        self._olly_position = Pose()
        self._olly_position_subscriber = self._position_subscriber = rospy.Subscriber(
            '/' + self._olly_name + "/position", PoseStamped, self._position_callback)

        self._olly_waypoint_subscriber = self._position_subscriber = rospy.Subscriber(
            '/' + self._olly_name + "/waypoint_queue", Pose, self._waypoint_queue_callback)

    def _position_callback(self, message):
        self._olly_position = message.pose

    def _waypoint_queue_callback(self, message):
        self.queue.append(message)

    def _publish_waypoint(self):
        self._olly_setpoint_publisher.publish(self._olly_setpoint)

    def _publish_next_waypoint(self):
        self._olly_setpoint = self.queue.pop(0)
        self._publish_waypoint()

    def _assert_control(self):
        if len(self.queue) == 0:
            return
        distance =   (self._olly_position.position.x - self._olly_setpoint.position.x) ** 2 \
                   + (self._olly_position.position.y - self._olly_setpoint.position.y) ** 2
        if distance < self._tolerance_radius ** 2:
            self._publish_next_waypoint()

    def effectuate_queue(self):
        try:
            rate = rospy.Rate(1 / self._step_time)
            print("Main Loop executing for Waypoint Queue of %s" % self._olly_name)
            while not rospy.is_shutdown():
                self._assert_control()
                rate.sleep()

        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    controller_node = WaypointQueue()
    controller_node.effectuate_queue()
