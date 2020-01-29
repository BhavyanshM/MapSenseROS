import numpy as np
from mayavi import mlab


xy_min, xy_max = -10, 10

D = np.load('depth.npy')[:,:,2]
D = 255 - D
D = D/2

x_line = np.linspace(0, 48, 768)
y_line = np.linspace(0, 64, 1024)
A, B = np.meshgrid(y_line, x_line)

def fragment(i,j):
	x_min, x_max = i*16,(i+1)*16
	y_min, y_max = j*16,(j+1)*16
	X = A[x_min:x_max,y_min:y_max]
	Y = B[x_min:x_max,y_min:y_max]
	Z = D[x_min:x_max,y_min:y_max]
	print(x_min, x_max, y_min, y_max, Z.shape)
	return X,Y,Z

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

# for i in range(48):
# 	for j in range(6):
# 		X,Y,Z = fragment(i,j)


		# print(X.shape)
		# print(Y.shape)
		# print(Z.shape)

noise = np.random.normal(0,0.5,(768,1024))
sW = mlab.points3d(A,B,D+noise, scale_factor=0.1, color=(1,1,1))

mlab.show()