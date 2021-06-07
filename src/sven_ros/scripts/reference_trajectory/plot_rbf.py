#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
from trajectory import RadialBasisFunction as RBF

if __name__ == "__main__":
	rbf = RBF()
	x = np.arange(-3, 3+0.1, 0.1).tolist()
	y = []
	for i in x:
		y.append(rbf.evaluate(i))
	plt.plot(x,y)
	plt.show()
	
