import time 
import math

class PID_class:
    def __init__(self,kp,ki,kd) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sp = 0
        self.last_time = 0
        self.last_mv = None
        self.diff = 0
        self.acc = 0

    def set_constants(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.diff = 0
        self.acc = 0
        self.last_time = 0

    def set_setpoint(self,sp):
        self.sp = sp
    
    def calculate(self,mv):
        err = self.sp - mv
        dt = (time.time() - self.last_time)/1000
        if self.last_time:
            self.diff = (self.last_mv - mv)/dt
            self.acc+=err*dt
            if self.acc > 300:
                self.acc = 300
            elif self.acc < -300:
                self.acc = -300
        self.last_time = time.time()
        self.last_mv = mv
        PIDval = self.kp*err + self.ki*self.acc + self.kd*self.diff
        return math.floor(PIDval)