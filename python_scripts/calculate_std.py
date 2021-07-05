import numpy as np
import os

data = np.genfromtxt("../config/log/Graph1.txt",
                     names = True, delimiter = ',',dtype = None)

print("QuadGPSX:", np.std(data['QuadGPSX']) )


data = np.genfromtxt("../config/log/Graph2.txt",
                     names = True, delimiter = ',',dtype = None)

print("QuadIMUAX std:",np.std(data['QuadIMUAX']) )


