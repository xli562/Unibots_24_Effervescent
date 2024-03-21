import numpy as np


a = np.array([[1,2,3],[3,4,5]])


def polar_to_cartesian(r: float, theta: float):
    """ Convert polar coordinates to cartesian coordinates. """
    x = r * np.cos(np.radians(theta))
    y = r * np.sin(np.radians(theta))
    return (x, y)

print(np.vectorize(polar_to_cartesian)(a[:, 1:]))