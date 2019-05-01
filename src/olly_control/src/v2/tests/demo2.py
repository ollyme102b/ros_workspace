import numpy as np
import matplotlib.pyplot as plt
import time
import sys

sys.path.append('../')
from FollyHighLevelControllerV2 import FollyHighLevelControllerV2

# Initialize Molly position
molly_position = np.array([0, 0])

# Initialize Folly position
folly_position = np.array([1, 0, 0])

# object length
object_length = 1

# line path constraint
path_constraints = np.array([[0, 1, -0.5]])

# simulation
T = 76  # Total time
dt = 0.5  # time per iteration
sigma = 0.0  # simulation noise standard deviation

# initialize MPC Alg
follyHLC = FollyHighLevelControllerV2(molly_position,
                                      folly_position,
                                      object_length,
                                      path_constraints,  # path_contraints
                                      50,  # horizon length
                                      dt,  # step time
                                      0.2  # maximum speed
                                      )

# constant molly velocity
molly_velocity_command = np.array([0, 0.05])

# total deviation
deviation = 0


# path_constaints plot function
def plot_constraints(path_constraints, x_domain=np.linspace(-1, 5, 10), **kwargs):
    for n in range(path_constraints.shape[0]):
        plt.plot(x_domain, (-path_constraints[n, 2] - path_constraints[n, 0] * x_domain) / path_constraints[n, 1],
                 **kwargs)


prev_time = time.time()  # time iteration for real time plot
new_time = time.time()
plt.figure(figsize=(32, 18))
plt.rc('font', size=30)
plt.show(block=False)
for t in range(T):
    load_length_deviation = np.linalg.norm(folly_position[0:2] - molly_position) - object_length
    deviation += np.abs(load_length_deviation)

    # plt.clf()
    # plot current state
    if t == 0:
        plt.plot([0.5,2],[0.5,0.5], color='black', linewidth=3, label='Road Edge')
        plt.plot([-0.5,-0.5],[2,-0.5], color='black', linewidth=3)
        plt.plot([-0.5, 2],[-0.5,-0.5], color='black', linewidth=3)
        plt.plot([0.5,0.5],[0.5,2],color='black', linewidth=3)
        #plot_constraints(path_constraints, label='Road Barrier for Folly', color='brown')  # constraint path
        plt.plot([molly_position[0], folly_position[0]], [molly_position[1], folly_position[1]], 'b-', linewidth=2,
                 label='Lifted Object Position')  # object lifted
        plt.plot(molly_position[0], molly_position[1], 'o', color='olive', label='Molly Position',
                 markersize=12)  # MollyPosition
        plt.plot(folly_position[0], folly_position[1], 'ro', label='Folly Position',
                 markersize=12)  # Folly Actual Positions
        plt.quiver(molly_position[0] - 0.25, molly_position[1], molly_velocity_command[0], molly_velocity_command[1],
                   color='olive', scale=.5)
    elif t % 15 == 0:
        plt.plot([molly_position[0], folly_position[0]], [molly_position[1], folly_position[1]], 'b-',
                 linewidth=2)  # object lifted
        plt.plot(molly_position[0], molly_position[1], 'o', color='olive', markersize=12)  # MollyPosition
        plt.plot(folly_position[0], folly_position[1], 'ro', markersize=12)  # Folly Actual Positions
    plt.legend()
    plt.axis('equal')
    plt.title(
        'Folly Following Molly on a Road')
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
    folly_position = folly_position + dt * folly_velocity_command + dt * np.random.normal(0, sigma, 3)

    if t == 50:
        follyHLC = FollyHighLevelControllerV2(molly_position,
                                              folly_position,
                                              object_length,
                                              horizon=50,  # horizon length
                                              step_time=dt,  # step time
                                              max_speed=0.2  # maximum speed
                                              )

print('The average deviation was {:.1f}cm.'.format(deviation / T * 100))
plt.savefig('samplefigure.png', dpi=100)
