#!/usr/bin/python

import numpy as np
import math
import random
print(__doc__)

from matplotlib import pyplot as plt

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from global_var import GRID, numOfActions
import global_var
from scipy.stats import multivariate_normal
from scipy.integrate import dblquad

np.random.seed(1)

numOfStates = (GRID * 2 + 1) * (GRID * 2 + 1) * (GRID * 2 + 1)

class update_transition_class:

	def upDate_transition(self,record,states,env):
		
		#T= [[[0 for I in xrange(numOfStates)] for J in xrange(numOfStates)] for K in xrange(numOfActions)]
		T = {}
		p = []

		actions = [(0,1,0),(0,-1,0),(1,0,0),(-1,0,0),(0,0,1),(0,0,-1)]
		#states = [ (i , j) for i in range(-GRID,GRID+1) for j in range(-GRID,GRID+1)]

		trainingX, targetX, trainingY, targetY, trainingZ, targetZ = [], [], [], [], [], []
		for elements in record:
			trainingX.append( [elements[0][0] , elements[0][1],elements[0][2] ,elements[1][0]] )
			targetX.append( elements[2][0] )
			
			trainingY.append( [elements[0][0], elements[0][1] ,elements[0][2] ,elements[1][1]] )
			targetY.append( elements[2][1] )  	

			trainingZ.append( [elements[0][0] , elements[0][1],elements[0][2] ,elements[1][2]] )
			targetZ.append( elements[2][2] )

		DX , tX = np.array( trainingX ), np.array( targetX )
		DY , tY = np.array( trainingY ), np.array( targetY )
		DZ , tZ = np.array( trainingZ ), np.array( targetZ )

		kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2))
		
		gpX = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9,alpha=1e-2)
		gpY = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9,alpha=1e-2)
		gpZ = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9,alpha=1e-2)
		
		gpX.fit(DX, tX)
		gpY.fit(DY, tY)
		gpZ.fit(DZ, tZ)


		for i in states:
			for k in actions:
				muX, sigmaX = gpX.predict( np.array( [i[0], i[1],i[2], k[0]] ), return_std=True, return_cov=False)
				muY, sigmaY = gpY.predict( np.array( [i[0], i[1],i[2], k[1]] ), return_std=True, return_cov=False)
				muZ, sigmaZ = gpZ.predict( np.array( [i[0], i[1],i[2], k[2]] ), return_std=True, return_cov=False)
				
				p[:] = []
				tempX = {(i[0],k[0]):round(sigmaX[0],3)}
				tempY = {(i[1],k[1]):round(sigmaY[0],3)}
				tempZ = {(i[2],k[2]):round(sigmaZ[0],3)}
				# print global_var.delta_t
				
				global_var.sigmaDictX = dict(global_var.sigmaDictX.items() + tempX.items())
				global_var.sigmaDictY = dict(global_var.sigmaDictY.items() + tempY.items())
				global_var.sigmaDictZ = dict(global_var.sigmaDictZ.items() + tempZ.items())

				#temporary = {(i , k): (1, (max(min(a,GRID),-GRID) , max(min(b,GRID),-GRID) ) ) }
				
				stateValue = np.array([ i[0] +  muX[0] * global_var.delta_t, i[1] + muY[0] * global_var.delta_t], i[2] +  muZ[0] * global_var.delta_t)
				
				# For neighbor states
				neighborXmin = max(min(i[0]-2,GRID),-GRID)
				neighborXmax = max(min(i[0]+3,GRID),-GRID)
				neighborYmin = max(min(i[1]-2,GRID),-GRID)
				neighborYmax = max(min(i[1]+3,GRID),-GRID)
				neighborZmin = max(min(i[2]-2,GRID),-GRID)
				neighborZmax = max(min(i[2]+3,GRID),-GRID)

				neighborStates = [ (itr1, itr2, itr3) for itr1 in range(neighborXmin,neighborXmax) for itr2 in range(neighborYmin,neighborYmax) for itr3 in range(neighborZmin,neighborZmax)]
				
				for j in neighborStates:
					#prob = self.probability_of_states(j[0],j[1],mu[0],sigma*np.eye(2))
					prob = self.probability_of_states(j[0], j[1],j[2], stateValue, [[sigmaX[0]**2, 0,0],[0, sigmaY[0]**2,0],[0,0,sigmaZ[0]**2]])
					p.append(round(prob,3))

				p_norm = [round(float(probabilities)/(sum(p) + 0.01), 3)  for probabilities in p]
				list_of_prob = zip(neighborStates, p_norm)
				temporary = {(i , k): list_of_prob}
				
				T = dict(T.items() + temporary.items() )
				#print T
		return T
	'''
	Integrate Gaussian over the rectangle.

	'''					
	def probability_of_states(self, x, y, z,mu, variance):
		var = multivariate_normal (mean = mu, cov = variance)
		#print variance
		return var.pdf([x, y, z]) 

		

	