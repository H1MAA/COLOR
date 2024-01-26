#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Int32MultiArray, String
from sensor_msgs.msg import Joy, Image
import numpy as np
import model
# import movement
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics_ros.msg import YoloResult

# import controller
# import mechanisms
import subprocess

freq = 30
r = 7
l = 50.5

angle = 0
motion_flag = bool

differential = model.Differential(r, l)
current_speeds = np.zeros(differential.wheels)

def imu_rotation(data):
    global angle
    angle = data.data

# def YOLO_Callback(data):
#     # This is the function to compute necessary commands #
#     # to preproccess the data and compute the movement   #
    

def initial_mapping():
    # move robot forward for 1 second
    # rotate robot 360 degrees
    pass

def adjust_callback(data):
    global current_speeds
    omega = data.data
    current_speeds =  differential.calc_speeds(0,0,omega)
    pass
    
intial_map_flag = True
current_speeds = np.zeros(differential.wheels)

if __name__ == '__main__':
    rospy.init_node("controller", anonymous=True)
    rospy.Subscriber("imu/yaw/deg", Float32, imu_rotation)
    rospy.Subscriber("adjust", Float32, adjust_callback)
    setpoints = rospy.Publisher("setpoints", Float32MultiArray, queue_size=10)
    # rospy.Subscriber("ultralytics_ros/yolo_result", Image, YOLO_Callback)
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        if intial_map_flag:
            initial_mapping()
            intial_map_flag = False
        setpoints.publish(Float32MultiArray(data = current_speeds))
        rate.sleep()