import numpy as np
import cvxpy


class CFTOCSolverV0:
    def calculate_optimal_actuation(self, x0, xbar):
        """
        Updates x0 and xbar parameters then solves
        :param x0: updated x0
        :param xbar: updated xbar
        :return: optimal actuation
        """
        self.x0.value = x0
        self.xbar.value = xbar
        return self.solve_optimal_actuation()

    def solve_optimal_actuation(self):
        """
        solves cvxpy problem and returns problem solution variable
        :return: optimal actuation value
        """
        self.problem.solve(solver=cvxpy.ECOS)
        return self.U[:, 0].value

    def status(self):
        """
        returns problem status
        """
        return self.problem.status

    def __init__(self, A, B, x0, xbar, N, umax):
        """
        Initializes cvxpy problem with the following params
        :param A: state transition matrix
        :param B: control matrix
        :param x0: initial state
        :param xbar: prefered end state
        :param N: number of time steps per cftoc
        """

        nx = x0.shape[0]
        nu = B.shape[1]

        # assert types for debuging
        assert type(A) is np.ndarray, 'A must be a numpy array'
        assert type(B) is np.ndarray, 'B must be a numpy array'
        assert type(x0) is np.ndarray, 'x0 must be a numpy array'
        assert type(xbar) is np.ndarray, 'xbar must be a numpy array'
        assert type(N) is int, 'N must be an int'

        # assert problem requirements
        assert A.shape[0] == nx, 'row A must = number of x0 states'
        assert A.shape[0] == A.shape[1], 'A must be square'
        assert B.shape[0] == nx, 'row B must = number of x0 states'
        assert x0.ndim == 1, 'x0 must be flat'
        assert xbar.ndim == 1, 'xbar must be flat'
        assert len(x0) == len(xbar), 'x0 must be same lengtha s xbar'
        assert umax is None or umax > 0, 'umax must be None or greater than 0'

        # initialize cvxpy problem
        self.X = cvxpy.Variable((nx, N + 1))
        self.U = cvxpy.Variable((nu, N))
        self.x0 = cvxpy.Parameter(nx)
        self.xbar = cvxpy.Parameter(nx)

        # initialize Parameters
        self.x0.value = x0
        self.xbar.value = xbar

        # initialize constraints
        constraints = []

        # dynamics constraints
        constraints += [self.X[:, 0] == self.x0]
        for t in range(N):
            constraints += [self.X[:, t + 1] == A * self.X[:, t] + B * self.U[:, t]]

        # input constraints
        if umax is not None:
            self.umax = cvxpy.Parameter()
            self.umax.value = umax
            for t in range(N):
                constraints += [cvxpy.norm(self.U[:, t], 'inf') <= self.umax]

        # cost function initialization
        cost = 0

        # state cost
        for t in range(1, N + 1):
            cost += cvxpy.sum_squares(self.X[:, t] - self.xbar)

        # input cost
        for t in range(N):
            cost += cvxpy.sum_squares(self.U[:, t])

        self.problem = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
