import numpy as np
from CFTOCSolverV1 import CFTOCSolverV1
from circleLineIntersectionPath import circle_line_intersection_path


class FollyHighLevelControllerV1:
    """
    Folly High Level Controller Version 1
    """

    def __init__(self, molly_initial_position=np.zeros((2,)),
                 folly_initial_position=np.zeros((3,)),
                 object_length=1,
                 line_path_start_point=np.array([0, 0]),
                 line_path_end_point=np.array([1, 0]),
                 horizon=10,
                 step_time=0.1,
                 max_speed=0.1):
        """
        Initialized controller
        :param molly_initial_position: molly initial position
        :param folly_initial_position: folly initial position
        :param object_length: lifted object length
        :param line_path_start_point: Folly path line constraint start point
        :param line_path_end_point: Folly path line constraint end point
        :param horizon: MPC horizon
        :param step_time: time in between MPC steps
        :param max_speed: maximum allowable Folly speed
        """
        self._object_length = object_length
        self._line_path_start_point = line_path_start_point
        self._line_path_end_point = line_path_end_point
        self._horizon = horizon
        self._step_time = step_time

        folly_desired_path = np.zeros((3, self._horizon))
        folly_desired_path[0:2, :] = self._desired_path(molly_initial_position, folly_initial_position, np.zeros((2,)))

        A = np.eye(3)  # state dynamics
        B = step_time * np.eye(3)  # input velocity dynamics

        self.optimizer = CFTOCSolverV1(A, B, folly_initial_position, folly_desired_path, horizon, max_speed)

    def _desired_path(self, molly_position, folly_position, molly_velocity):
        """
        computes Folly desired path analytically as interstection of circle and line over time steps
        :param molly_position: current molly position as center of circle
        :param folly_position: current folly position to deduce better of two circle intersect line solutions
        :return: folly desired path
        """
        d_m_pos = molly_velocity * self._step_time
        folly_desired_path = circle_line_intersection_path(molly_position,
                                                           self._object_length,
                                                           self._line_path_start_point,
                                                           self._line_path_end_point,
                                                           d_m_pos,
                                                           self._horizon,
                                                           f_pos=folly_position
                                                           )
        return np.array(folly_desired_path)

    def update_then_calculate_optimal_actuation(self, current_molly_position, current_folly_position,
                                                current_molly_velocity):
        """
        calculates optimal input for controller
        :param current_molly_position: current molly position
        :param current_folly_position: current folly position
        :param current_molly_velocity: current molly velocity
        :return: optimal velocity command
        """
        folly_desired_path = np.zeros((3, self._horizon))  ###### SETS A ZERO YAW POSITION SET POINT
        folly_desired_path[0:2, :] = self._desired_path(current_molly_position, current_folly_position[0:2],
                                                        current_molly_velocity)
        return self.optimizer.calculate_optimal_actuation(current_folly_position, folly_desired_path)
