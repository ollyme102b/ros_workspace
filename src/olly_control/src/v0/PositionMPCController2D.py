import numpy as np

import imp

CFTOCSolverV0 = imp.load_source('CFTOCSolverV0',
                                '/home/jasonanderson/ME102B_Project/ros_workspace/src/olly_control/src/v0/CFTOCSolverV0.py')

from CFTOCSolverV0 import CFTOCSolverV0


class PositionMPCController2D:
    """
    Basic 2D Position MPC Controller
    """

    def __init__(self, olly_initial_position=np.zeros((2,)), horizon=10, step_time=0.1, max_speed=0.1):
        """
        Initialized controller
        :param olly_initial_position: olly initial position
        :param horizon: MPC horizon
        :param step_time: time in between MPC steps
        :param max_speed: maximum allowable Folly speed
        """
        self._horizon = horizon
        self._step_time = step_time

        A = np.eye(2)  # state dynamics
        B = step_time * np.eye(2)  # input velocity dynamics

        self.optimizer = CFTOCSolverV0(A, B, olly_initial_position, olly_initial_position, horizon, max_speed)

    def update_then_calculate_optimal_actuation(self, current_olly_position, current_olly_command):
        """
        calculates optimal input for controller
        :param current_olly_position: current olly position
        :return: optimal velocity command
        """
        return self.optimizer.calculate_optimal_actuation(current_olly_position, current_olly_command)
