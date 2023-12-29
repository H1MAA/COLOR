#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray, String
import numpy as np

class move:
    def __init__(self, model, wheels, v_1st, v_2nd, v_3rd, v_4th, omega_1st, omega_2nd, omega_3rd, omega_4th) -> None:
        self.model = model
        self.wheels = wheels
        self.start = True
        self.switch = True 
        self.vx = 0
        self.vy = 0
        self.vx_def = 0
        self.vy_def = 0
        self.omega = 0
        self.omega_def = 0
        self.set_point = []
        self.prev_smoothed = np.zeros(self.wheels)



        self.damping = 0.9
        self.deadzone = 0.1
        self.EPSILON = 1e-5
        self.use_rotational_matrix = False

        ### standard speeds ###
        self.v1 = v_1st
        self.v2 = v_2nd
        self.v3 = v_3rd
        self.v4 = v_4th
        self.omega1 = omega_1st
        self.omega2 = omega_2nd
        self.omega3 = omega_3rd
        self.omega4 = omega_4th

        self.pub_speeds = rospy.Publisher("speeds_setpoints", Float32MultiArray, queue_size=10)
        self.pub_debug_speeds = rospy.Publisher("debug_speeds", Float32MultiArray, queue_size=10)
   
    def movement(self, ctrl, motion_flag):
        self.motion_flag = motion_flag
        if self.start:
            if ctrl.rt and ctrl.lt:
                self.start = False
                print("done")
            else:
                self.vx = 0
                self.vy = 0
                self.omega = 0
                return
        
        #######   speed control  ###########
        if ctrl.controls["RB"] and not ctrl.controls["LB"]:
            self.vx_def = self.v4
            self.vy_def = self.v4
            self.omega_def = self.omega4
        elif ctrl.controls["LB"] and not ctrl.controls["RB"]:
            self.vx_def = self.v3
            self.vy_def = self.v3
            self.omega_def = self.omega3
        elif ctrl.controls["RB"] and  ctrl.controls["LB"]:
            self.vx_def = self.v1
            self.vy_def = self.v1
            self.omega_def = self.omega1
        elif not ctrl.controls["RB"] and not ctrl.controls["LB"]:
            self.vx_def = self.v2
            self.vy_def = self.v2
            self.omega_def = self.omega2
        ########## direction control  ##########
        if self.motion_flag:
            if abs(ctrl.controls["L_horz"]):
                self.vy = int(ctrl.controls['L_horz'] * self.vy_def)
            else:
                self.vy = 0

            if abs(ctrl.controls["L_vert"]):
                self.vx = int(ctrl.controls['L_vert'] * self.vx_def)
            else:
                self.vx = 0
                
            if ctrl.controls["RT"] < 1:
                self.omega = (ctrl.controls["RT"]-1)*self.omega_def
            elif ctrl.controls["LT"] < 1:
                self.omega = -(ctrl.controls["LT"]-1)*self.omega_def
            else:
                self.omega = 0
        else:
            if ctrl.controls["RT"] < 1:
                self.omega = (ctrl.controls["RT"]-1)*self.omega_def
            elif ctrl.controls["LT"] < 1:
                self.omega = -(ctrl.controls["LT"]-1)*self.omega_def
            else:
                self.omega = 0
        debug_speeds = Float32MultiArray()
        debug_speeds.data = np.array([self.vx,self.vy,self.omega])
        self.pub_debug_speeds.publish(debug_speeds)
    
    def get_speeds(self, angle, prev_smoothed):
        speeds = self.model.calc_speeds(self.vx, self.vy, self.omega, self.use_rotational_matrix, angle)
        msg = Float32MultiArray()
        msg.data = tuple(self.smooth(speeds, prev_smoothed))
        self.pub_speeds.publish(msg)
        return msg.data

    def smooth (self,data ,prev_smoothed):
        current_speed = data
        current_smoothed = np.zeros(self.wheels,np.float32)
        for i in range(self.wheels):
            if abs(current_speed[i]) > self.EPSILON:
                current_smoothed[i] = self.damping * prev_smoothed[i] + (1-self.damping) * current_speed[i]

            if abs(current_smoothed[i]) < self.deadzone:
                current_smoothed[i] = 0
        return current_smoothed

    def mpc_pub(self):
        mpc = Float32MultiArray()
        mpc.data = tuple(self.set_point)
        self.pub_mpc.publish(mpc)