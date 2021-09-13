#!/usr/bin/python3

import json
import numpy as np
from datalib import *
from models import *

class ProMPReader(object):
	"""docstring for RosbagReader."""

	def __init__(self, prompfile, **kwargs):
		self.prompfile = prompfile
		self.promp_handles, self.rotation_matrix = self.read()

	def read(self):
		f = open(self.prompfile,'r')
		data = json.load(f)
		
		result = []
		
		for phase in data['phases']:
			pos_promp_dicts = phase['position_promps']
			or_promp_dicts = phase['orientation_promps']
			promps = []
			for i in range(len(pos_promp_dicts)):
				pos_promp = ProMPHandler()
				pos_promp.from_dict(pos_promp_dicts[i])
				promps.append(pos_promp)
			for i in range(len(or_promp_dicts)):
				or_promp = ProMPHandler()
				or_promp.from_dict(or_promp_dicts[i])
				promps.append(or_promp)
			result.append(promps)
		
		return result, np.array(data['rotation_matrix'])
			
		
