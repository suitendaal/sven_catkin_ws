import json

from .movement_primitives.promp_handler import ProMPHandler

class EndEffectorHandler(object):

	def __init__(self, filename):
		self.filename = filename
		
		self.phase = 0
		self.n_phases = 0
		self.pos_promps = []
		self.or_promps = []
		
		self.setup()
		
	def setup(self):
		
		f = open(self.filename)
		json_object = json.load(f)
		self.from_dict(json_object)
		f.close()
		
	def update_phase(self):
		self.phase += 1
		
	def evaluate(self, time):
		x = self.pos_promps[self.phase][0].evaluate(time)[0][0]
		y = self.pos_promps[self.phase][1].evaluate(time)[0][0]
		z = self.pos_promps[self.phase][2].evaluate(time)[0][0]
		qx = self.or_promps[self.phase][0].evaluate(time)[0][0]
		qy = self.or_promps[self.phase][1].evaluate(time)[0][0]
		qz = self.or_promps[self.phase][2].evaluate(time)[0][0]
		qw = self.or_promps[self.phase][3].evaluate(time)[0][0]
		
		xd = self.pos_promps[self.phase][0].evaluate(time, derivative=1)[0][0]
		yd = self.pos_promps[self.phase][1].evaluate(time, derivative=1)[0][0]
		zd = self.pos_promps[self.phase][2].evaluate(time, derivative=1)[0][0]
	
		return [x, y, z, qx, qy, qz, qw, xd, yd, zd]
		
	def get_impact_intervals(self):
		impact_intervals = []
		if self.n_phases > 1:
			for i in range(self.n_phases-1):
				if len(self.pos_promps[i]) > 0:
					impact_interval = []
					impact_interval.append(self.pos_promps[i+1][0].get_extended_start_end()[0])
					impact_interval.append(self.pos_promps[i][0].get_extended_start_end()[1])
					impact_intervals.append(impact_interval)
		return impact_intervals
		
	def get_time_interval(self):
		time_interval = []
		if self.n_phases > 0 and len(self.pos_promps[0]) > 0:
			time_interval.append(self.pos_promps[0][0].get_extended_start_end()[0])
			time_interval.append(self.pos_promps[-1][0].get_extended_start_end()[1])
		return time_interval
		
	def from_dict(self, json_object):
	
		# Number of phases
		self.n_phases = json_object['n_phases']
		
		# Position ProMPs
		self.pos_promps = []
		for i in json_object['pos_promps']:
			pos_promps_phase = []
			for j in i:
				promp_handler = ProMPHandler()
				promp_handler.from_dict(j)
				pos_promps_phase.append(promp_handler)
			self.pos_promps.append(pos_promps_phase)
			
		
		# Orientation ProMPs
		self.or_promps = []
		for i in json_object['or_promps']:
			or_promps_phase = []
			for j in i:
				promp_handler = ProMPHandler()
				promp_handler.from_dict(j)
				or_promps_phase.append(promp_handler)
			self.or_promps.append(or_promps_phase)
	

