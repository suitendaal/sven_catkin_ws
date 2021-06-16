#!/usr/bin/python3

import numpy as np
from datalib import *

class ProMP(object):
	"""docstring for ProMP."""
	
	def __init__(self, basis_functions, **kwargs):
		self.basis_functions = basis_functions
		self.weights = kwargs.get('weights',np.zeros((len(self.basis_functions),1)))
		self.weights_covariance = kwargs.get('weights_covariance',1E0 * np.eye(len(self.basis_functions)))
		self.derivatives = kwargs.get('derivatives',0)
		
	def learn(self, datasets, **kwargs):
		self.weights_covariance = kwargs.get('weights_covariance',self.weights_covariance)
	
		self.weights = np.zeros((len(self.basis_functions),len(datasets)))
		
		for i in range(len(datasets)):
			dataset = datasets[i]
	
			# Calculate outcome of basis functions
			time_vector = np.array(dataset.time())
			psi = np.zeros((len(self.basis_functions),len(time_vector) * (self.derivatives + 1)))
			
			for k in range(self.derivatives + 1):
				for j in range(len(time_vector)):
					index = j * (self.derivatives + 1) + k
					psi[:,index] = self.calc_phi(time_vector[j], derivative=k)[:,0]
					
			# Calculate weights for each demonstration
			values = dataset.values()
			
			if self.derivatives > 0:
				values = [item for sublist in values for item in sublist[0:(self.derivatives+1)]]
			x = np.transpose(np.array([values]))
			pseud = np.transpose(np.linalg.pinv(psi))
			self.weights[:,i] = pseud.dot(x)[:,0]
		
		# Calculate mean and covariance
		Mu_w = np.mean(self.weights,axis=1)
		Sigma_w = self.weights_covariance
		
		if self.weights.shape[1] > 1:
			Sigma_w = Sigma_w + np.cov(self.weights)
		
		Mu = np.transpose(psi).dot(Mu_w)
			
		return Mu_w,Sigma_w,Mu
		
	def calc_phi(self, time, derivative=0, normalize=True):
		phi = np.zeros((len(self.basis_functions),1))
		if not normalize or derivative == 0:
			for i in range(len(self.basis_functions)):
				phi[i,0] = self.basis_functions[i].evaluate(time,derivative=derivative)
		if normalize:
			if derivative == 0:
				phi[:,0] = phi[:,0] * 1 / np.sum(phi[:,0])
			elif derivative == 1:
				zero_order = self.calc_phi(time, derivative=0, normalize=False)
				first_order = self.calc_phi(time, derivative=1, normalize=False)
				phi[:,0] = ((first_order * np.sum(zero_order[:,0]) - zero_order * np.sum(first_order[:,0])) * 1 / (np.sum(zero_order[:,0]) ** 2))[:,0]
		return phi
		
	def mu_w(self):
		return np.mean(self.weights,axis=1)
	
	def sigma_w(self):
		Sigma_w = self.weights_covariance
		
		if self.weights.shape[1] > 1:
			Sigma_w = Sigma_w + np.cov(self.weights)
		
		return Sigma_w
		
	def evaluate(self, time, derivative=0, **kwargs):
		via_points = kwargs.get('via_points',DataSet())
		if not isinstance(time, list):
			time = [time]
		
		index = len(time)
		time.extend(via_points.time())
		
		Mu_w = self.mu_w()
		Sigma_w = self.sigma_w()
		
		phi = np.zeros((len(self.basis_functions),len(time)))
			
		for i in range(index):
			phi[:,i] = self.calc_phi(time[i],derivative=derivative)[:,0]
		for i in range(index,len(time)):
			derivative = 0
			j = i - index
			if isinstance(via_points[j], ViaPoint):
				derivative = via_points[j].derivative
			phi[:,i] = self.calc_phi(time[i],derivative=derivative)[:,0]
			
		Mu = np.transpose(phi).dot(Mu_w)
		Sigma = np.transpose(phi).dot(Sigma_w).dot(phi)
		
		if len(via_points) > 0:
			Mu_out = Mu[:index]
			Sigma_out_in = Sigma[:index,index:]
			Sigma_in = Sigma[index:,index:]
			Sigma_out = Sigma[:index,:index]
			Mu_in = np.transpose(np.array(via_points.values())) - Mu[index:]
			Mu = Mu_out + Sigma_out_in.dot(np.linalg.inv(Sigma_in)).dot(Mu_in)
			
			# TODO: Sigma
		
		return Mu,Sigma
		
