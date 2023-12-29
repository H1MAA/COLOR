#!/usr/bin/env python
import numpy as np

class Model:
    def __init__(self,kinematic_model, inverse_model) -> None:
        self.kinematic_model = kinematic_model
        self.inverse_model = inverse_model
        self.wheels = kinematic_model.shape[0]

    def get_kinematic(self):
        return self.kinematic_model
    
    def calc_speeds(self, vx, vy, omega, rotational_flag = False, rotatational_theta = 0):
        if rotational_flag:
            rotational = np.array([
                    [np.cos(np.radians(rotatational_theta)),-np.sin(np.radians(rotatational_theta)),0],
                    [np.sin(np.radians(rotatational_theta)),np.cos(np.radians(rotatational_theta)),0],
                    [0,0,1]])
        else:
            rotational = np.identity(3)

        differential = np.array([vx, vy, omega])
        rotated_differential = rotational @ differential
        speeds = self.kinematic_model @ rotated_differential
        speeds = speeds / 2 * np.pi
        return speeds
    def calc_pose():
        pass

class Differential(Model):
    def __init__(self, r, l) -> None:
        self.r = r
        self.l = l
        kinematic_model = np.array([[(1/self.r), (self.l/(2*self.r))],
                                   [(1/self.r), (-self.l/(2*self.r))]
                                   ])
        super().__init__(kinematic_model)
