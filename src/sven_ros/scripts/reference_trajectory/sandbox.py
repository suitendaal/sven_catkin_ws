#!/usr/bin/python3

from datalib import *

dataset = DataSet([],timefactor=10)
dataset.align_time()
for i in range(5):
	dataset.append(DataPoint(i+10,[2*i, 3*i, 4*i]))
	
print(dataset)
print(dataset.diff())
