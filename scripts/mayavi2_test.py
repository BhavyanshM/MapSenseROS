import numpy as np
from subprocess import call
from mayavi import mlab

degree = 3
lx, ly = 16, 16
xy_min, xy_max = -5, 5

x_line = np.linspace(xy_min, xy_max, 16)
y_line = np.linspace(xy_min, xy_max, 16)
X, Y = np.meshgrid(x_line, y_line)

u_line = np.linspace(xy_min, xy_max, 30)
v_line = np.linspace(xy_min, xy_max, 30)
U, V = np.meshgrid(u_line, v_line)


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
			total += (poly(P,X[i,j],Y[i,j]) - Z[i,j])**2
	return total

def grad(f, Z, P, gdel):
	gd = np.zeros_like(P)
	for i in range(len(P)):
		dp = np.zeros_like(P)
		dp[i] = gdel 
		pos, neg = P + dp, P - dp
		# print(pos, neg)
		gd[i] = (f(Z, pos) - f(Z, neg))/(2*gdel)
	return gd


W = cubic(U,V)
# W_n = W + np.random.normal(0,5,W.shape)

Z = cubic(X,Y)
Z_n = Z + np.random.normal(0,5,Z.shape)

# View it.

P = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
alpha = 0.000001
gdel = 0.01

np.savetxt("data.txt", Z_n)

print("Running poly.c")
call(["gcc", "-o", "poly", "poly.c", "-lm"])
call(["./poly", "args"])

while True:

	# Compute the residual
	r = residual(Z,P)

	# if residual exceeds threshold
	# print(r)
	if r > 10000:
		# compute gradient
		g = grad(residual, Z_n, P, gdel)

		# move parameters along gradient
		P -= alpha*g

	# else
	else:
		# stop
		break
# print(P)

G = P[0]*U**3 + P[1]*U**2 + P[2]*U**1 + P[3]*V**3 + P[0]*V**2 + P[0]*V**1 + P[0]*V**0

print(G.shape)


sW = mlab.points3d(X, Y, Z_n/100, scale_factor=0.1, color=(1,1,1))
sWN = mlab.mesh(U, V, G/100)
# mlab.show()