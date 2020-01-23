import numpy as np
from mayavi import mlab

degree = 3
lx = 16, ly = 16
xy_min, xy_max = -5, 5

x_line = np.linspace(xy_min, xy_max, 16)
y_line = np.linspace(xy_min, xy_max, 16)
X, Y = np.meshgrid(x_line, y_line)

u_line = np.linspace(xy_min, xy_max, 30)
v_line = np.linspace(xy_min, xy_max, 30)
U, V = np.meshgrid(u_line, v_line)

W = cubic(U,V)
# W_n = W + np.random.normal(0,5,W.shape)

Z = cubic(X,Y)
Z_n = Z + np.random.normal(0,5,Z.shape)

def biquad(X,Y):
	return X**4 + 3*Y**3 + X**2 + Y**2 + 10

def cubic(X,Y):
	return Y**3 - 10*X**2 - Y**2 + 1

def basis(X,Y):
	return np.array([X**3, X**2, X, Y**3, Y**2, Y, 1])

def poly(P, X, Y):
	return np.dot(P,basis(X,Y))

def residual(Z, P):
	total = 0
	for i in range(lx):
		for j in range(ly):
			total += poly(P,X[i,j],Y[i,j]) - Z[i,j]
	return total

def gradient(P):
	for i in range(degree):
		# calc gradient


# View it.
sW = mlab.points3d(X, Y, Z_n/100, scale_factor=0.1, color=(1,1,1))
sWN = mlab.mesh(U, V, W/100)
mlab.show()

P = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
while True:

	# Compute the residual
	r = residual(Z,P)

	# if residual exceeds threshold
	if r > 0.1:
		# compute gradient
		grad = gradient(P)

		# move parameters along gradient
		P += alpha*grad

	# else
	else:
		# stop
		break

