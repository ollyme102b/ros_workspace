import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
from src.localization.ins_kalman_filter import linear_gpsimu_pos_kalman_filter

x0_mean = np.zeros((9, 1))
x0_cov = 0.1 * np.eye(9)
kf = linear_gpsimu_pos_kalman_filter(1, x0_mean, x0_cov)

state = np.zeros((9, 1))

a_var = 0.1
p_var = 0.01

square = lambda t: (((t + 3) % 12 < 6) - ((t + 3) % 12 >= 6))*(t < 24)

for t in range(1000):
    state[6] = 0.1 * square(t)  # square-wave acceleration in x direction

    state[0:6] = state[0:6] + state[3:9]

    measurement = np.zeros((9, 1))
    measurement[0:3] = state[0:3] + p_var * np.random.randn(3, 1)  # measure position with GPS
    measurement[6:9] = state[6:9] + a_var * np.random.randn(3, 1)  # measure acceleration with IMU

    kf_estimate, kf_estimate_cov = kf.estimate(measurement)

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(state[0], state[1], 'b.')
    ax.plot(measurement[0], measurement[1], 'r.')
    ax.plot(kf_estimate[0], kf_estimate[1], 'g.')

    w, v = np.linalg.eig(kf_estimate_cov[0:2, 0:2])
    ax.add_patch(Ellipse((kf_estimate[0], kf_estimate[1]), w[0], w[1], np.arctan2(v[0, 1], v[0, 0]), alpha=0.5))

    plt.legend(['Actual', 'Measurement', 'KF Estimate', 'KF Confidence Interval'])
    plt.xlabel('X Pos')
    plt.ylabel('Y Pos')
    plt.title('t = {0}'.format(t))
    plt.xlim((-0.5, 1.5))
    plt.ylim((-1, 1))
    plt.show(block=False)
    plt.pause(1)
    plt.close()
