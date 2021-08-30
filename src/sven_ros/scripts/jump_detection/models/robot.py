from datalib import *
from models import *
from readers import *

class Robot(object):
	def __init__(self, config):
		self.config_ = config
		self.phases = []
		self.promps = []
		self.init_phases()
		
	@property
	def n_phases(self):
		result = 0
		if len(self.config_.impact_intervals) > 0:
			result = len(self.config_.impact_intervals[0])
		return result
		
	@property
	def demos(self):
		return self.config_.demos
	
	@property
	def position_filter(self):
		return self.config_.position_filter
		
	@property
	def orientation_filter(self):
		return self.config_.orientation_filter
		
	@property
	def velocity_filter(self):
		return self.config_.velocity_filter
		
	@property
	def rotational_velocity_filter(self):
		return self.config_.rotational_velocity_filter
		
	def get_starting_index(self, phase, demo_index):
		start = 0
		if phase > 0:
			start = self.config_.impact_intervals[demo_index][phase-1][-1] + 1
		return start
	
	def get_ending_index(self, phase_index, demo_index):
		end = len(position_data[0])
		if phase_index < len(self.config_.impact_intervals[demo_index]):
			end = self.config_.impact_intervals[demo_index][phase_index][0]
		return end
		
	def get_starting_time(phase_index):
		# TODO
		return 0
		
	def get_ending_time(phase_index):
		# TODO
		return 0
		
	def init_phases(self):
		# Read data
		position_data, orientation_data, velocity_data, rotational_velocity_data = [], [], [], []
		for i in range(len(self.demos)):
			position, orientation, velocity, rotational_velocity = self.read_demo_data(self.demos[i])
			position_data.append(position)
			orientation_data.append(orientation)
			velocity_data.append(velocity)
			rotational_velocity_data.append(rotational_velocity)
	
		for i in range(self.n_phases):
			
			# List with phase of each demonstration
			self.phases.append([])
			
			for j in range(len(self.demos)):
				# Initialize data of phase
				start = self.get_starting_index(i, j)
				end = self.get_ending_index(i, j)
					
				filtered_position_datasets = []
				filtered_orientation_datasets = []
				filtered_velocity_datasets = []
				filtered_rotational_velocity_datasets = []
				
				for k in range(3):
					filtered_position_datasets.append(FilteredDataSet(position_data[j][k][start:end], self.position_filter))
					filtered_orientation_datasets.append(FilteredDataSet(orientation_data[j][k][start:end], self.orientation_filter))
					filtered_velocity_datasets.append(FilteredDataSet(velocity_data[j][k][start:end], self.velocity_filter))
					filtered_rotational_velocity_datasets.append(FilteredDataSet(rotational_velocity_data[j][k][start:end], self.rotational_velocity_filter))
					
				self.phases[i].append(Phase(filtered_position_datasets, filtered_orientation_datasets, filtered_velocity_datasets, filtered_rotational_velocity_datasets))
					
		
	def read_demo_data(self, demo):
		position = []
		orientation = []
		velocity = []
		rotational_velocity = []
		
		for i in range(3):
			position.append(DataSet())
			orientation.append(DataSet())
			velocity.append(DataSet())
			rotational_velocity.append(DataSet())
			
		franka_reader = FrankaStateReader(demo)
		while not franka_reader.end():
			dp = franka_reader.next_datapoint()
			time = dp.time
			franka_state = dp.value
			
			for j in range(3):
				position[j].append(DataPoint(time, franka_state.position[j]))
				orientation[j].append(DataPoint(time, franka_state.euler_angles[j]))
				velocity[j].append(DataPoint(time, franka_state.velocity[j]))
				rotational_velocity[j].append(DataPoint(time, franka_state.rotational_velocity[j]))
				
		return position, orientation, velocity, rotational_velocity
	
	def create_promps(self):
		for i in range(self.n_phases):
			# Create radial basis functions
			rbfs = self.create_basis_functions(i)
		
	def create_basis_functions(self, phase_index):
		starting_time = self.get_starting_time(phase_index)
		ending_time = self.get_ending_time(phase_index)
