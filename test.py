import numpy as np

# Example matrix: a 90-degree rotation matrix
M = np.array([[0, -1], 
              [1, 0]])

app = np.array([2,3])

print(np.append(M, app))