import numpy as np

a = np.array([4,3,2,1,5,6,7,8])

i = np.argmin(a[6:])
print(i+6)