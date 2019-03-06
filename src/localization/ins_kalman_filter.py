import numpy as np
import cv2 as cv
from abc import ABC


class ins_pos_kalman_filter(ABC):
    def __init__(self, F, Q, H, R, initial_state_mean, initial_state_covariance):
        """
        abstract initialization of kalman filter for INS data fusion for position estimation
        Matrix notation matches that provided by https://en.wikipedia.org/wiki/Kalman_filter
        :param F: state transition model matrix
        :param Q: process noise covariance matrix
        :param H: observation model matrix
        :param R: observation noise covariance matrix
        """
        if type(F) is not np.ndarray:
            raise TypeError('F matrix must by np.ndarray')
        if type(Q) is not np.ndarray:
            raise TypeError('Q matrix must by np.ndarray')
        if type(H) is not np.ndarray:
            raise TypeError('H matrix must by np.ndarray')
        if type(R) is not np.ndarray:
            raise TypeError('R matrix must by np.ndarray')

        if F.shape[1] != H.shape[1]:
            raise RuntimeError('F and H must have same number of columns')
        if Q.shape[1] != R.shape[1]:
            raise RuntimeError('Q and R must have same number of columns')

        self._kf = cv.KalmanFilter(F.shape[1], Q.shape[1])

        self._kf.transitionMatrix = F
        self._kf.processNoiseCov = Q
        self._kf.measurementMatrix = H
        self._kf.measurementNoiseCov = R
        self._kf.statePost = initial_state_mean
        self._kf.errorCovPost = initial_state_covariance

    def estimate(self, measurement):
        """
        incorporates measurment into kalman filter to update estimate and returns the current estimate provided by the
        kalman filter
        :param measurement: the measurement from sensors
        :return: the estimate state
        :return: the estimate state covariance
        """
        self._kf.predict()
        self._kf.correct(measurement)
        return self._kf.statePost, self._kf.errorCovPost


class linear_gpsimu_pos_kalman_filter(ins_pos_kalman_filter):
    def __init__(self, T, x0_mean, x0_cov):
        """
        initializes linear kalman filter that fuses GPS and IMU sensors with linear transition matrices
        :param T: time step in between estimations
        """
        if type(T) not in [int, float]:
            raise TypeError('T must be a number')

        I3 = np.eye(3)
        O3 = np.zeros((3, 3))
        B = np.array([[1.], [1.], [1.]])

        F = np.block([[I3, T * I3, T ** 2 / 2 * I3],
                      [O3, I3, T * I3],
                      [O3, O3, I3]])
        Q = np.diag(np.hstack([T ** 3 / 6 * B.T, T ** 2 / 2 * B.T, T * B.T]).flatten())
        H = np.block([[I3, O3, O3],
                      [O3, O3, O3],
                      [O3, O3, I3]])
        R = np.eye(9)  # THIS IS A PLACE HOLDER, REPLACE WITH NOISE COV OF GPS AND IMU SENSORS

        super().__init__(F, Q, H, R, x0_mean, x0_cov)
