#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray, String
from sensor_msgs.msg import Joy, Image
import numpy as np
import model 
import movement
# import controller
# import mechanisms
import subprocess

freq = 30
r = 7
l = 50.5

angle = 0

v_1st = 10
v_2nd = 20
v_3rd = 32
v_4th = 45

omega_1st = 0.2
omega_2nd = 0.4
omega_3rd = 0.8
omega_4th = 1.2

motion_flag = bool


differential = model.Differential(r, l)
# ctrl = controller.Controller()
move = movement.move(differential, differential.wheels, v_1st, v_2nd, v_3rd, v_4th, omega_1st, omega_2nd, omega_3rd, omega_4th)
# mech = mechanisms.Mechanisms()

current_speeds = np.zeros(differential.wheels)

def imu_rotation(data):
    global angle
    angle = data.data

def YOLO_Callback(data):
    # This is the function to compute necessary commands #
    # to preproccess the data and compute the movement   #
    
    pass

def initial_mapping():
    # move robot forward for 1 second
    # rotate robot 360 degrees
    pass


intial_map_flag = True

if __name__ == '__main__':
    
    rospy.init_node("controller", anonymous=True)
    rospy.Subscriber("imu/yaw/deg", Float32, imu_rotation)
    rospy.Subscriber("ultralytics_ros/YoloResult", Image, YOLO_Callback)
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        if intial_map_flag:
            initial_mapping()
            intial_map_flag = False
        

        # def initial_mapping
        current_speeds = move.get_speeds(angle, current_speeds)
        rate.sleep()