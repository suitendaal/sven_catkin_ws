import numpy as np

class ProMP(object):
	"""docstring for ProMP."""
	
	def __init__(self, basis_functions, **kwargs):
		self.basis_functions = basis_functions
		self.weights = kwargs.get('weights',np.zeros((len(self.basis_functions),1)))
		self.weights_covariance = kwargs.get('weights_covariance',1E0 * np.eye(len(self.basis_functions)))
		if isinstance(self.weights_covariance, int):
			self.weights_covariance = self.weights_covariance * np.eye(len(self.basis_functions))
		self.derivatives = kwargs.get('derivatives',0)
		
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
		# [{'time', 'value', 'derivative'}]
		via_points = kwargs.get('via_points',[])
		if not isinstance(time, list):
			time = [time]
		else:
			time = time.copy()
		
		index = len(time)
		for via_point in via_points:
			time.append(via_point['time'])
		
		Mu_w = self.mu_w()
		Sigma_w = self.sigma_w()
		
		phi = np.zeros((len(self.basis_functions),len(time)))
			
		# Requested time
		for i in range(index):
			phi[:,i] = self.calc_phi(time[i],derivative=derivative)[:,0]
			
		# Via points
		for i in range(index,len(time)):
			j = i - index
			derivative = via_points[j]['derivative']
			phi[:,i] = self.calc_phi(time[i],derivative=derivative)[:,0]
			
		Mu = np.transpose(phi).dot(Mu_w)
		Sigma = np.transpose(phi).dot(Sigma_w).dot(phi)
		
		if len(via_points) > 0:
			Mu_out = Mu[:index]
			Sigma_out_in = Sigma[:index,index:]
			Sigma_in = Sigma[index:,index:]
			Sigma_out = Sigma[:index,:index]
			
			via_point_values = []
			for via_point in via_points:
				via_point_values.append(via_point['value'])
			Mu_in = np.transpose(np.array(via_point_values)) - Mu[index:]
			Mu = Mu_out + Sigma_out_in.dot(np.linalg.inv(Sigma_in)).dot(Mu_in)
			
			# TODO: Sigma
		
		return Mu,Sigma
		
	def to_dict(self):
		json_object = dict()
		
		# Basis function
		json_object['basis_functions'] = []
		for i in self.basis_functions:
			json_object['basis_functions'].append(i.to_dict())
		json_object['derivatives'] = self.derivatives
			
		# Weights
		json_object['weights'] = self.weights.tolist()
		json_object['weights_covariance'] = self.weights_covariance.tolist()
			
		return json_object
		
	def from_dict(self, json_object):
		self.derivatives = json_object['derivatives']
			
		# Weights
		self.weights = np.array(json_object['weights'])
		self.weights_covariance = np.array(json_object['weights_covariance'])
		
