import rospy
from std_msgs.msg import Float32 , Int16, Float32MultiArray, Int16MultiArray, String, Byte

class Control():
    def __init__(self):
        rospy.init_node('test')
        self.adjust= rospy.Publisher('adjust', Float32, queue_size=100)    

    def kill(self):
        self.adjust.publish(Float32(data=0))
        rospy.loginfo("Kill")

    def set_omega(self, omega):
        self.adjust.publish(Float32(data=omega))
        rospy.loginfo(f"Set Omega to {omega}")

