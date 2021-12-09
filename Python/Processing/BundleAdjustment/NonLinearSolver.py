import numpy as np
np.set_printoptions(precision=2)

class NonLinearSolver:
    def __init__(self, num_params):
        self.params = np.zeros(shape=(num_params))
        self.Jacobian = None
        print(self.params)

    def GetCameraJacobian(self, point):
        dg = np.array([[0, -point[2], point[1], -1, 0, 0],
                  [point[2], 0, -point[0], 0, -1, 0],
                  [-point[1], point[0], 0, 0, 0, -1]])
        return self.GetProjectionJacobian(point) @ dg

    def GetPointJacobian(self, point, R):
        return self.GetProjectionJacobian(point) @ R.T

    def GetProjectionJacobian(self, point):
        xp = - point[0] / point[2]
        yp = - point[1] / point[2]
        return 1/point[2] * (np.array([[1, 0, -xp],[0, 1, -yp]]))

    def Linearize(self):
        # Build the large Jacobian for the current Linearization point
        A = np.zeros(shape=(40,40))
        for i in range(10):
            for j in range(10):
                if i == j:
                    A[i*4:(i+1)*4, j*4:(j+1)*4] = np.random.normal(3, 1, (4,4))

        self.Jacobian = A
        return self.Jacobian


    def Solve(self):
        pass

    def Update(self):
        pass

