"""
class PID
attributes:
    kp, 
    ki, 
    kd 

methods?
compute_error(des, actual) -> return error 
get_gains() -> return command


"""
import numpy as np

class PID():
    def __init__(self, kp, ki, kd, dt) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.error = [0,0]
        self.errors = 0.0

    def compute_error(self, des:float, actual:float):
        self.error[0] = des-actual
        if self.error[0] > np.pi:
            self.error[0] = self.error[0] - 2*np.pi

        if self.error[0] < -np.pi:
            self.error[0] = self.error[0] + 2*np.pi
        return self.error[0]
        
    def get_gains(self, des, actual):
        #p gains
        
        self.compute_error(des, actual)
        p = self.kp * self.error[0]
        area = ((self.error[0]-self.error[1])/2)*self.dt

        d = self.kd * (self.error[0] - self.error[1])/self.dt

        i = self.ki*(area + self.errors)
        #update error
        self.error[1] = self.error[0]
        self.errors = self.errors + area

        
        return p+i+d