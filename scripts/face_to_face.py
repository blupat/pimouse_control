#!/usr/bin/env python
import rospy, math, sys
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class FaceToFace():
    __slots__ = ('__cmd_vel', '__angular_gain')

    def __init__(self):
        self.__cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.__angular_gain = rospy.get_param("/vision_control/angular_gain", 0.3)
    
    def Rotate(self, xPosRate):
        rot = -self.__angular_gain * xPosRate * math.pi
        
        m = Twist()
        m.linear.x = 0.0
        m.angular.z = rot
        self.__cmd_vel.publish(m)

if __name__ == "__main__":
    rospy.init_node("face_to_face")
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.ServiceProxy('motor_on', Trigger).call()
    fd = FaceToFace()
    fd.Rotate(float(sys.argv[1]))
    rospy.sleep(3)
    rospy.ServiceProxy('motor_off', Trigger).call()
