import scipy.sparse.linalg
from scipy import sparse
import numpy as np
np.set_printoptions(precision=2)

from TransformUtils import *
from Camera import *

class NonLinearSolver:
    def __init__(self, num_params):
        self.params = np.zeros(shape=(num_params,))
        self.Jacobian = None
        self.iterations = 10
        self.cam = Camera()

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

        transform = np.eye(4)
        self.cam.SetTransform(transform)

        # Build vector of residuals for all observations at current Linearization point
        b = np.zeros(shape=(200,))
        # TODO: Complete the vector above.
        for i in range(2):
            for j in range(50):
                P = np.array(self.params[12 + j*3 : 12 + (j+1)*3])
                P = np.array([P[0], P[1], P[2], 1])
                # print(i, j, "Point:", P)
                h = self.cam.Project(P)
                b[i*100 + j*2 : i*100 + (j+1)*2] = h - measurements[i*50 + j , :]


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

    def Compute(self, measurements, initial):

        self.params = initial
        for i in range(self.iterations):

            A, b = self.Linearize(self.params, measurements)
            total_err = np.sum(np.abs(b))

            # if total_err < 10:
            #     break

            dx = self.Solve(A, b)
            self.params = self.Update(self.params, np.array(dx))

            print("Iteration: ", i, "Total Error: ", total_err, "Update:", dx[:12])


