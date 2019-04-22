import numpy as np
import matplotlib.pyplot as plt
import time

from importlib.machinery import SourceFileLoader

FollyHighLevelControllerV2 = SourceFileLoader('FollyHighLevelControllerV2',
                                              '../FollyHighLevelControllerV2.py').load_module()
from FollyHighLevelControllerV2 import FollyHighLevelControllerV2

# Initialize Molly position
molly_position = np.array([-4, -8])

# Initialize Folly position
folly_position = np.array([-2, -7.5])

# object length
object_length = 2

# line path constraint
path_constraints = np.array([[4, 10, 80]])

# simulation
T = 150  # Total time
dt = 0.5  # time per iteration
sigma = 0.0  # simulation noise standard deviation

# initialize MPC Alg
follyHLC = FollyHighLevelControllerV2(molly_position,
                                      folly_position,
                                      object_length,
                                      path_constraints,  # path_contraints
                                      10,  # horizon length
                                      dt,  # step time
                                      0.1  # maximum speed
                                      )

# constant molly velocity
molly_velocity_command = np.array([0.01, 0.05])

# total deviation
deviation = 0


# path_constaints plot function
def plot_constraints(path_constraints, x_domain=np.linspace(-10, 0, 10)):
    for n in range(path_constraints.shape[0]):
        plt.plot(x_domain, (-path_constraints[n, 2] - path_constraints[n, 0] * x_domain) / path_constraints[n, 1])


prev_time = time.time()  # time iteration for real time plot
new_time = time.time()
plt.show(block=False)
for t in range(T):
    load_length_deviation = np.linalg.norm(folly_position - molly_position) - object_length
    deviation += np.abs(load_length_deviation)

    plt.clf()
    # plot current state
    plot_constraints(path_constraints)  # constraint path
    plt.plot([molly_position[0], folly_position[0]], [molly_position[1], folly_position[1]], 'b-', linewidth=2,
             label='Load: dev {:.1f}cm'.format(load_length_deviation * 100))  # object lifted
    plt.plot(molly_position[0], molly_position[1], '.', color='olive', label='Molly Position')  # MollyPosition
    plt.plot(folly_position[0], folly_position[1], 'r.', label='Folly Position')  # Folly Actual Positions
    plt.legend()
    plt.axis('equal')
    plt.title(
        'Folly High Level Controller V2 {0}/{1} Iteration Time {2:.2f}s'.format(t, T,
                                                                                follyHLC.optimizer.m.options.SOLVETIME))
    # plt.title(
    #    'Folly High Level Controller V2 {0}/{1} Iteration Time {2:.2f}s'.format(t, T, new_time - prev_time))
    plt.xlabel('[m]')
    plt.ylabel('[m]')

    plt.pause(0.01)
    time.sleep(np.maximum(dt - (new_time - prev_time), 0.01))  # pause to match real time
    prev_time = new_time
    new_time = time.time()

    # actuate molly command with noise
    molly_position = molly_position + dt * molly_velocity_command + dt * np.random.normal(0, sigma, 2)

    # compute folly command
    folly_velocity_command = follyHLC.update_then_calculate_optimal_actuation(molly_position, folly_position,
                                                                              molly_velocity_command)

    # actuate optimal command with noise
    folly_position = folly_position + dt * folly_velocity_command + dt * np.random.normal(0, sigma, 2)

print('The average deviation was {:.1f}cm.'.format(deviation / T * 100))
