class Phase(object):
	def __init__(self, position_datasets, orientation_datasets, velocity_datasets, rotational_velocity_datasets):
		self.position_data = position_datasets
		self.orientation_data = orientation_datasets
		self.velocity_data = velocity_datasets
		self.rotational_velocity_data = rotational_velocity_datasets
		
