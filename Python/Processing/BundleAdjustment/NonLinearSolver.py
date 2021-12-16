import scipy.sparse.linalg
from scipy import sparse
import numpy as np
np.set_printoptions(precision=2, suppress=True)

from TransformUtils import *
from Camera import *

class NonLinearSolver:
    def __init__(self, P, C, N):
        self.params = np.zeros(shape=(P,))
        self.Jacobian = None
        self.iterations = 10
        self.cam = Camera()
        self.C = C
        self.P = P
        self.N = N
        self.error = 1000000000000

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

    def CalculateError(self, measurements):
        b = np.zeros(shape=(2 * self.C * self.N,))
        for i in range(self.C):
            for j in range(self.N):
                P = np.array(self.params[12 + j*3 : 12 + (j+1)*3])
                P = np.array([P[0], P[1], P[2], 1])
                # print(i, j, "Point:", P)
                h = self.cam.Project(P)
                b[i * (2 * self.N) + j * 2: i * (2 * self.N) + (j + 1) * 2] = h - measurements[i * self.N + j , :]
        return b

    def Linearize(self, params):

        transform = np.eye(4)
        self.cam.SetTransform(transform)

        # Build the large Jacobian for the current Linearization point
        A = np.zeros(shape=(2 * self.C * self.N, self.P))

        # Set Camera Jacobian in first few columns of A
        for i in range(self.C): # No. of Cameras
            for j in range(self.N): # No. of 3D Points
                if i != 0:
                    A[i*(2*self.N) + j*2 : (i*(2*self.N) + (j+1)*2), i*6 : (i+1)*6] = self.GetCameraJacobian(self.params[12 + j*3 : 12 + (j+1)*3])

        # Set Point Jacobian in remaining columns of A
        for i in range(self.C): # No. of Cameras
            for j in range(self.N): # No. of 3D Points
                A[i*(2*self.N) + j*2 : (i*(2*self.N) + (j+1)*2), 12 + j*3 : (12 + j*3 + 3)] = self.GetPointJacobian(self.params[12 + j*3 : 12 + (j+1)*3],
                                                                                                      SO3Exp(self.params[i*6 : (i*6 + 3)]))

        self.Jacobian = A
        return A


    def Solve(self, A, b):
        sA = sparse.csr_matrix(A)
        x = scipy.sparse.linalg.lsqr(A, b)
        return x[0]

    def Update(self, params, dx):
        self.params += dx
        return self.params

    def Compute(self, measurements, initial, true_points):

        self.params = initial
        total_err = 0
        for i in range(self.iterations):


            # Calculate Jacobian at the current linearization point
            A = self.Linearize(self.params)

            # Build vector of residuals for all observations at current Linearization point
            b = self.CalculateError(measurements)
            total_err = np.sum(np.abs(b))

            if total_err < self.error:
                self.error = total_err
            else:
                break

            print(i, "Error: ", total_err, "Params: ", self.params[6:12])

            dx = self.Solve(A, b)
            self.params = self.Update(self.params, np.array(dx))





