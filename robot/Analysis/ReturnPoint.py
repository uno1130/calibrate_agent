import numpy as np

class ReturnPointAnalyzer:
    def __init__(self):
        self.returnVelocity = []

    def analyze(self, velocityList):
        velocity_arr = np.array(velocityList)
        minIndices = np.where((velocity_arr[1:-1] < velocity_arr[:-2]) & (velocity_arr[1:-1] < velocity_arr[2:]))[0] + 1
        minIndices_alpha = minIndices - 50

        for index in minIndices_alpha:
            self.returnVelocity.append(velocity_arr[index])
        returnVelocity_mean = np.mean(self.returnVelocity)
        return returnVelocity_mean