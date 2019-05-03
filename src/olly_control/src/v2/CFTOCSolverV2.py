import numpy as np
import gekko


class CFTOCSolverV2:
    def calculate_optimal_actuation(self, x0, Xm):
        """
        Updates x0 and Xm parameters and returns problem solution solves
        :param x0: updated x0
        :param Xm: updated Xm
        :return: optimal actuation
        """
        self.update_problem(x0, Xm)
        return self.solve_optimal_actuation()

    def update_Xm(self, Xm):
        """
        update Xm parameter
        """
        for t in range(self.N):
            for i in range(self.nx):
                self.Xm[i, t].value = Xm[i, t]
        return

    def update_x0(self, x0):
        """
        update x0 parameter

        """
        for i in range(self.nx):
            self.x0[i].value = x0[i]
        return

    def update_problem(self, x0, Xm):
        """
        Updates x0 and Xm parameters
        :param x0: updated x0
        :param Xm: updated Xm
        """
        self.update_x0(x0)
        self.update_Xm(Xm)
        return

    def solve_optimal_actuation(self):
        """
        solves cvxpy problem and returns problem solution variable
        :return: optimal actuation value
        """
        self.m.solve(disp=False)
        for i in range(self.N):
            if np.array(self.slack[i,0].value)[0] > 0.001:
                print(np.array(self.slack[i,0].value)[0])
        return np.array([self.U[0, 0].value, self.U[1, 0].value, self.U[2, 0].value]).flatten()

    def status(self):
        """
        returns problem status
        """
        return self.m.options.SOLVESTATUS

    def __init__(self, A, B, x0, Xm, N, umax, l, path_constraints):
        """
        Initializes cvxpy problem with the following params
        :param A: state transition matrix
        :param B: control matrix
        :param x0: initial state
        :param Xm: expected molly path
        :param N: number of time steps per cftoc
        """

        self.nx = x0.shape[0]
        self.nu = B.shape[1]
        self.N = N

        # assert types for debuging
        assert type(A) is np.ndarray, 'A must be a numpy array'
        assert type(B) is np.ndarray, 'B must be a numpy array'
        assert type(x0) is np.ndarray, 'x0 must be a numpy array'
        assert type(Xm) is np.ndarray, 'xbar must be a numpy array'
        assert type(N) is int, 'N must be an int'

        # assert problem requirements
        assert A.shape[0] == self.nx, 'row A must = number of x0 states'
        assert A.shape[0] == A.shape[1], 'A must be square'
        assert B.shape[0] == self.nx, 'row B must = number of x0 states'
        assert x0.ndim == 1, 'x0 must be flat'
        assert umax is None or umax > 0, 'umax must be None or greater than 0'

        # create GEKKO object
        self.m = gekko.GEKKO(remote=False)
        self.m.options.SOLVER = 2

        # initialize GEKKO Vars
        self.X = self.m.Array(self.m.Var, (self.nx, N + 1))
        self.U = self.m.Array(self.m.Var, (self.nu, N))
        self.slack = self.m.Array(self.m.Var, (self.N, 1))

        # initilaize GEKKO Params
        self.x0 = self.m.Array(self.m.Param, (self.nx,))
        for i in range(self.nx):
            self.x0[i].value = x0[i]
        self.Xm = self.m.Array(self.m.Param, (self.nx, N))
        for i in range(self.nx):
            for t in range(N):
                self.Xm[i, t].value = Xm[i, t]

        # dynamic constraints
        for i in range(self.nx):
            self.m.Equation(self.X[i, 0] == self.x0[i])
        for t in range(N):
            temp = np.dot(A, self.X[:, t]) + np.dot(B, self.U[:, t])
            for i in range(self.nx):
                self.m.Equation(self.X[i, t + 1] == temp[i])

        #slack constraints
        for t in range(N):
            self.m.Equation(self.slack[t,0] >= 0)

        # path constraints
        if path_constraints is not None:
            for n in range(path_constraints.shape[0]):
                for t in range(N):
                    self.m.Equation(np.dot(path_constraints[n, 0:2], self.X[0:2, t]) + path_constraints[n, 2] - self.slack[t,0]<= 0)

        # input constraints
        if umax is not None:
            self.umax = self.m.Param(value=umax)
            for t in range(N):
                for i in range(self.nu):
                    self.m.Equation(self.U[i, t] ** 2 <= self.umax ** 2)

        # initialize cost function
        J = 0

        # state cost
        for t in range(1, N + 1):
            J +=  10*((self.X[0, t] - self.Xm[0, t - 1]) ** 2 + (self.X[1, t] - self.Xm[1, t - 1]) ** 2 - l ** 2) ** 2
            J += self.X[2, t] ** 2
        for t in range(N):
            J += 100*self.slack[t,0]

        # input cost
        for t in range(N):
            for i in range(self.nu):
                J += self.U[i, t] ** 2

        # set problem objective function
        self.m.Obj(J)

        # reduce overhead time by disabling web output
        self.m.options.WEB = 0