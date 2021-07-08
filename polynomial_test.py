import numpy as np


times = [0, 1, 2, 3, 4, 5]
values = [2, 3, 4, 5, 4, 3]
w = [1, 2, 3, 3, 2, 1]

coefs = np.polyfit(times, values, 3, w=w)
print(coefs)

result = []
for i in times:
	result.append(np.polyval(coefs, i))
print(result)

#T = [
#	[1, 0, 0, 0],
#	[1, 1, 1, 1],
#	[1, 2, 4, 8],
#	[1, 3, 9, 27],
#	[1, 4, 16, 64],
#	[1, 5, 25, 125]
#]

#print(np.linalg.pinv(T))


