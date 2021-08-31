from .demo_variable import *
from .promp_handler import *
from trajectory import *
import numpy as np

class RobotVariable(object):
	def __init__(self, datasets, derivative_datasets, impact_intervals):
		self.demo_variables = []
		for i in range(len(datasets)):
			demo_data = DemoVariable(datasets[i], derivative_datasets[i], impact_intervals[i])			
			self.demo_variables.append(demo_data)
			
	@property
	def n_phases(self):
		if len(self.demo_variables) > 0:
			return self.demo_variables[0].n_phases
		return 0
		
	def filter_data(self, data_filter):
		for demo_data in self.demo_variables:
			demo_data.filter_data(data_filter)
	
	def filter_derivative(self, derivative_filter):
		for demo_data in self.demo_variables:
			demo_data.filter_derivative(derivative_filter)
			
	def get_starting_time(self, phase):
		t_start = None
		for demo_data in self.demo_variables:
			if t_start is None or demo_data.get_starting_time(phase) + demo_data.time_shift(phase) > t_start:
				t_start = demo_data.get_starting_time(phase) + demo_data.time_shift(phase)
		return t_start
	
	def get_ending_time(self, phase):
		t_end = None
		for demo_data in self.demo_variables:
			if t_end is None or demo_data.get_ending_time(phase) + demo_data.time_shift(phase) < t_end:
				t_end = demo_data.get_ending_time(phase) + demo_data.time_shift(phase)
		return t_end
		
	def get_real_starting_time(self, phase):
		t_start = None
		for demo_data in self.demo_variables:
			if t_start is None or demo_data.get_real_starting_time(phase) + demo_data.time_shift(phase) > t_start:
				t_start = demo_data.get_real_starting_time(phase) + demo_data.time_shift(phase)
		return t_start
		
	def get_real_ending_time(self, phase):
		t_end = None
		for demo_data in self.demo_variables:
			if t_end is None or demo_data.get_real_ending_time(phase) + demo_data.time_shift(phase) < t_end:
				t_end = demo_data.get_real_ending_time(phase) + demo_data.time_shift(phase)
		return t_end
	
	def get_phase_starting_time(self, phase):
		t_start = None
		for demo_data in self.demo_variables:
			if t_start is None or demo_data.get_phase_starting_time(phase) + demo_data.time_shift(phase) > t_start:
				t_start = demo_data.get_phase_starting_time(phase) + demo_data.time_shift(phase)
		return t_start
		
	def get_phase_ending_time(self, phase):
		t_end = None
		for demo_data in self.demo_variables:
			if t_end is None or demo_data.get_phase_ending_time(phase) + demo_data.time_shift(phase) < t_end:
				t_end = demo_data.get_phase_ending_time(phase) + demo_data.time_shift(phase)
		return t_end
		
	def get_extended_starting_time(self, phase):
		t_start = None
		for demo_data in self.demo_variables:
			if t_start is None or demo_data.get_extended_starting_time(phase) + demo_data.time_shift(phase) > t_start:
				t_start = demo_data.get_extended_starting_time(phase) + demo_data.time_shift(phase)
		return t_start
		
	def get_extended_ending_time(self, phase):
		t_end = None
		for demo_data in self.demo_variables:
			if t_end is None or demo_data.get_extended_ending_time(phase) + demo_data.time_shift(phase) < t_end:
				t_end = demo_data.get_extended_ending_time(phase) + demo_data.time_shift(phase)
		return t_end
			
	def extend_data(self, extender):
	
		# Determine starting time of extended phase
		if len(self.demo_variables) > 0:
			for phase in range(self.demo_variables[0].n_phases):
				t_start = self.get_starting_time(phase)
				t_end = self.get_ending_time(phase)
				for variable in self.demo_variables:
					variable.set_phase_time(phase, t_start, t_end)
	
		for demo_data in self.demo_variables:
			demo_data.extend_data(extender)
			
	def create_phase_rbfs(self, phase, width, rbfs_per_second):
		t_start = self.get_extended_starting_time(phase)
		t_end = self.get_extended_ending_time(phase)
		n_rbfs = round(rbfs_per_second * (t_end - t_start) + 1)
		rbfs = []
		for i in range(n_rbfs):
			center = (t_end - t_start) * i / (n_rbfs-1) + t_start
			rbf = RadialBasisFunction(center=center, width=width)
			rbfs.append(rbf)
		
		return rbfs
		
	def create_promps(self, rbf_width, rbfs_per_second):
		promps = []
		for phase in range(self.n_phases):
			rbfs = self.create_phase_rbfs(phase, rbf_width, rbfs_per_second)
			promps.append(self.create_phase_promp(phase, rbfs))
		
		# Jump intervals
		if self.n_phases > 1:
			for phase in range(self.n_phases):
				if phase < self.n_phases - 1:
					jump_intervals = []
					for demo_data in self.demo_variables:
						jump_intervals.append(demo_data.get_data(phase+1)[0].time - demo_data.get_data(phase)[-1].time)
					promps[phase].next_starting_time = np.mean(jump_intervals)
				if phase > 0:
					jump_intervals = []
					for demo_data in self.demo_variables:
						jump_intervals.append(demo_data.get_data(phase)[0].time - demo_data.get_data(phase-1)[-1].time)
					promps[phase].previous_ending_time = np.mean(jump_intervals)
		
		# Align promps
		for phase in range(self.n_phases):
			if phase == 0:
				time_align = -promps[phase].extended_starting_time
			else:
				time_align = promps[phase-1].get_phase_start_end()[-1] - promps[phase].phase_starting_time + promps[phase].previous_ending_time
			promps[phase].align_time(time_align)
			
		return promps
		
	def create_phase_promp(self, phase, rbfs):
		t_start_phase = self.get_phase_starting_time(phase)
		t_end_phase = self.get_phase_ending_time(phase)
		t_start_real = self.get_real_starting_time(phase)
		t_end_real = self.get_real_ending_time(phase)
		t_start_extended = self.get_extended_starting_time(phase)
		t_end_extended = self.get_extended_ending_time(phase)
		
		datasets = []
		for demo_data in self.demo_variables:
			dataset = DataSet()
			position_data = demo_data.get_extended_data(phase)
			velocity_data = demo_data.get_extended_derivative(phase)
			time_shift = demo_data.time_shift(phase)
			for i in range(len(position_data)):
				dataset.append(DataPoint(position_data[i].time + time_shift, [position_data[i].value, velocity_data[i].value]))
			datasets.append(dataset)
		
		promp = ProMP(rbfs,derivatives=1,weights_covariance=1)
		promp.learn(datasets)
		mp_handler = ProMPHandler(promp, phase_starting_time=t_start_phase, phase_ending_time=t_end_phase, starting_time=t_start_real, ending_time=t_end_real, extended_starting_time=t_start_extended, extended_ending_time=t_end_extended)
		
		return mp_handler
				
	
