# -*- coding: utf-8 -*-
"""
Created on Jan 29 2017
@author: Xiaoqiang Liu
"""


import numpy as np
from numpy import dot, array, sqrt
import sympy
from sympy import symbols, Matrix
from sympy.abc import r, c, theta, beta
from filterpy.kalman import ExtendedKalmanFilter as EKF RAOD


class RoadEKF(EKF):
	def __init__(self, dt): 
		EKF.__init__(self, 4, 2)
		r, c , theta, beta = symbols('r, c, theta, beta') # beta is the change of theta
		self.dt = dt
		self.fxu = Matrix([[r - dt*sympy.sin(theta + beta*dt/2)],
			[c + dt*sympy.cos(theta + beta*dt/2)],
			[theta + dt*beta],
			[beta]])
			
		self.F_j = self.fxu.jacobian(Matrix([r, c, theta, beta]))
		self.subs = {r:0, c:0, theta:0, beta:0}
		self.r, self.c, self.theta, self.beta = r, c, theta, beta
	
	def predict(self, u=0):
		
		# predict the next state
		dx = np.array([[self.dt*np.sin(x[2,0] + x[3,0]*dt/2.0)],
			[self.dt*np.cos(x[2,0] + x[3,0]*dt/2.0)],
			[x[3,0]*self.dt],
			[0]])
		
		self.x += dx
		
		self.subs[self.r] = self.x[0,0]
		self.subs[self.c] = self.x[1,0]
		self.subs[self.theta] = self.x[2,0]
		self.subs[self.beta] = self.beta[3,0]
		F = array(self.F_j.evalf(subs = self.subs)).astype(float)	
		
		self.P = np.dot(F, self.P).dot(F.T) + self.Q

		
	def update(self, z):
	    
		P = self.P
		R = self.R
		if np.isscalar(z) and self.dim_z == 1:
		    z = np.asarray([z], float)
		
		x = self.x
		H = self.H
		PHT = dot(P, H.T)
		S = dot(H, PHT) + R
		
		self.K = PHT.dot(linalg.inv (S))
		self.y = z - np.dot(H, x)
		self.x = x + dot(self.K, self.y)
		
		I_KH = self._I - dot(self.K, H)
		self.P = dot(I_KH, P).dot(I_KH.T) + dot(self.K, R).dot(self.K.T)
		
		
		