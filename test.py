import numpy as np

# Example matrix: a 90-degree rotation matrix
M = np.array([[0, -1], 
              [1, 0]])

# Example array of vectors
V = np.array([[1, 0], 
              [0, 1], 
              [1, 1]])

# Apply the matrix to every vector in V
transformed_V = V @ M.T

print(transformed_V)
