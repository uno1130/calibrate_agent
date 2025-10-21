import numpy as np
from scipy.signal import find_peaks

class ReturnPointAnalyzer:
    def __init__(self):
        self.returnVelocity = []

    # def analyze(self, velocityList):
    #     velocity_arr = np.array(velocityList)
    #     minIndices = np.where((velocity_arr[1:-1] < velocity_arr[:-2]) & (velocity_arr[1:-1] < velocity_arr[2:]))[0] + 1
    #     minIndices_alpha = minIndices

    #     for index in minIndices_alpha:
    #         self.returnVelocity.append(velocity_arr[index])
    #     returnVelocity_mean = np.mean(self.returnVelocity)
    #     return returnVelocity_mean
    
    def analyze(self, velocityList):
        velocity_arr = np.array(velocityList)
        
        inverted_velocity = -velocity_arr
        minIndices, _ = find_peaks(inverted_velocity, distance=50)
        
        for index in minIndices:
            self.returnVelocity.append(velocity_arr[index])
        
        # print("Detected return points at indices:", self.returnVelocity)
        return self.returnVelocity