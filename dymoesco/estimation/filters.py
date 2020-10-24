import numpy as np

class KalmanFilter():

	def __init__(self, A, B, C, Q, R):
		self.A = A
		self.B = B
		self.C = C
		self.Q = Q
		self.R = R

	def predict(self, x, P, u):
		x = self.A @ x + self.B @ u
		P = self.A @ P @ self.A.T + self.Q
		return x, P

	def update(self, x, P, z):
		y = z - self.C @ x
		K = P @ self.C.T @ np.linalg.pinv(self.C @ P @ self.C.T + self.R)
		x = x + K @ y
		P = (np.eye(P.shape[0]) - K @ self.C) @ P
		return x, P

class EKF():
    
    def __init__(self, f, g, Q, R):
        self.f = f
        self.F = jacobian(f)
        self.g = g
        self.G = jacobian(g)
        self.Q = Q
        self.R = R

    def predict(self, x, P, u):
        x = self.f(x, u)
        P = self.F(x,u) @ P @ self.F(x,u).T + self.Q
        return x, P
        
    def update(self, x, P, z):
        y = z - self.g(x)
        Gx = self.G(x)
        K = self.P @ Gx @ np.linalg.pinv(Gx @ self.P @ Gx.T + self.R)
        x = self.x + K@y
        P = (np.eye(P.shape[0]) - K @ G) @ P
        return x, P