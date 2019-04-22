import numpy as np

from importlib.machinery import SourceFileLoader

CFTOCSolverV2 = SourceFileLoader('CFTOCSolverV2', '../CFTOCSolverV2.py').load_module()
from CFTOCSolverV2 import CFTOCSolverV2


class FollyHighLevelControllerV2:
    """
    Folly High Level Controller Version 2
    """

    def __init__(self, molly_initial_position=np.zeros((2,)),
                 folly_initial_position=np.zeros((2,)),
                 object_length=1,
                 path_constraints=None,
                 horizon=10,
                 step_time=0.5,
                 max_speed=1):
        """
        Initialized controller
        :param molly_initial_position: molly initial position
        :param folly_initial_position: folly initial position
        :param object_length: lifted object length
        :param path_constraints: path constaints in specific format
        :param horizon: MPC horizon
        :param step_time: time in between MPC steps
        :param max_speed: maximum allowable Folly speed
        """
        self._object_length = object_length
        self._path_constraints = path_constraints
        self._horizon = horizon
        self._step_time = step_time

        molly_expected_path = self._molly_expected_path(molly_initial_position, np.zeros((2,)))

        A = np.eye(2)  # state dynamics
        B = step_time * np.eye(2)  # input velocity dynamics

        self.optimizer = CFTOCSolverV2(A, B, folly_initial_position, molly_expected_path, horizon, max_speed,
                                       object_length, path_constraints)

    def _molly_expected_path(self, molly_position, molly_velocity):
        """
        computes expected molly path assuming molly velocity is constant
        :param molly_position: current molly position
        :param molly_velocity: current molly velocity
        :return: folly desired path
        """
        molly_expected_path = np.zeros((2, self._horizon))

        for t in range(self._horizon):
            molly_expected_path[:, t] = self._step_time * (t + 1) * molly_velocity + molly_position

        return molly_expected_path

    def update_molly_expectation(self, current_molly_position, current_molly_velocity):
        """
        updates solver params with current information regarding molly
        :param current_molly_position:
        :param current_molly_velocity:
        """
        molly_expected_path = self._molly_expected_path(current_molly_position, current_molly_velocity)
        self.optimizer.update_Xm(molly_expected_path)
        return

    def update_position(self, current_position):
        """
        update solver params with current information regarding folly
        :param current_position:
        """
        self.optimizer.update_x0(current_position)
        return

    def update_then_calculate_optimal_actuation(self, current_molly_position, current_folly_position,
                                                current_molly_velocity):
        """
        updates and calculates optimal input for controller
        :param current_molly_position: current molly position
        :param current_folly_position: current folly position
        :param current_molly_velocity: current molly velocity
        :return: optimal velocity command
        """
        self.update_molly_expectation(current_molly_position, current_molly_velocity)
        self.update_position(current_folly_position)
        return self.calculate_optimal_actuation()

    def calculate_optimal_actuation(self):
        """
        calculates optimal actuation as problem is currently
        :return: optimal actuation
        """
        return self.optimizer.solve_optimal_actuation()
