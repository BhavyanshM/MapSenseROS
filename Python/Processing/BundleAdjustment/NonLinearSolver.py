import scipy.sparse.linalg
from scipy import sparse
import numpy as np
np.set_printoptions(precision=2)

from TransformUtils import *

class NonLinearSolver:
    def __init__(self, num_params):
        self.params = np.ones(shape=(num_params,))
        self.Jacobian = None
        self.iterations = 10
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

    def Linearize(self, params, measurements):

        # Build vector of residuals for all observations at current Linearization point
        b = np.ones(shape=(200,))
        # TODO: Complete the vector above.

        # Build the large Jacobian for the current Linearization point
        A = np.zeros(shape=(200,162))

        # Set Camera Jacobian in first few columns of A
        for i in range(2): # No. of Cameras
            for j in range(50): # No. of 3D Points
                if i != 0:
                    A[i*100 + j*2 : (i*100 + (j+1)*2), i*6 : (i+1)*6] = self.GetCameraJacobian(self.params[12 + j*3 : 12 + (j+1)*3])

        # Set Point Jacobian in remaining columns of A
        for i in range(2): # No. of Cameras
            for j in range(50): # No. of 3D Points
                A[i*100 + j*2 : (i*100 + (j+1)*2), 12 + j*3 : (12 + j*3 + 3)] = self.GetPointJacobian(self.params[12 + j*3 : 12 + (j+1)*3],
                                                                                                      SO3Exp(self.params[i*6 : (i*6 + 3)]))

        self.Jacobian = A
        return A, b


    def Solve(self, A, b):
        sA = sparse.csr_matrix(A)
        x = scipy.sparse.linalg.lsqr(A, b)
        return x[0]

    def Update(self, params, dx):
        self.params += dx
        return self.params

    def Compute(self, measurements):
        for i in range(self.iterations):


            A, b = self.Linearize(self.params, measurements)

            total_err = np.sum(np.abs(b))
            # if total_err < 10:
            #     break

            dx = self.Solve(A, b)

            print("Iteration: ", i, "A: ", A.shape, "x: ", dx.shape, "b: ", b.shape, "Total Error: ", total_err)
            print("X: ", dx[:20])



            self.params = self.Update(self.params, np.array(dx))


