import numpy as np

def compute_path_length(x, y):
    length = 0
    for i in range(len(x)-1):
        dx = x[i+1] - x[i]
        dy = y[i+1] - y[i]
        length += np.sqrt(dx**2 + dy**2)
    return length
