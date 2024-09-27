import numpy as np

def norm_v(v):
    return v/np.linalg.norm(v)

def unit(angle):
    return np.array([np.cos(angle), np.sin(angle)])