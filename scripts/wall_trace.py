#!/usr/bin/env python
import rospy, copy, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallTrace():
    __slots__ = ('__cmdVel', '__accel', '__maxSpeed', '__minSpeed', '__servoTarget', '__servoKp', '__sensorValues', '__linearSpeed')
    
    def __init__(self):
        self.__cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.__accel = rospy.get_param("/run_corridor/acceleration", 0.01)
        self.__maxSpeed = rospy.get_param("/run_corridor/max_speed", 0.3)
        self.__minSpeed = rospy.get_param("/run_corridor/min_speed", 0.0)
        self.__servoTarget = rospy.get_param("/run_corridor/servo_target", 50)
        self.__servoKp = rospy.get_param("/run_corridor/servo_kp", 1.0)
        
        self.__sensorValues = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.Callback)
        
        self.__linearSpeed = 0.0

    def Callback(self, messages):
        self.__sensorValues = messages

    def Run(self):
        data = Twist()
        
        s = self.__sensorValues
        self.__linearSpeed += self.__accel
        
        if s.sum_forward >= 50:
            self.__linearSpeed = 0.0
        elif self.__linearSpeed <= self.__minSpeed:
            self.__linearSpeed = self.__minSpeed
        elif self.__linearSpeed >= self.__maxSpeed:
            self.__linearSpeed = self.__maxSpeed
        data.linear.x = self.__linearSpeed
        
        if data.linear.x < self.__minSpeed:
            data.angular.z = 0.0
        elif s.left_side < 10:
            data.angular.z = 0.0
        else:
            error = self.__servoTarget - s.left_side
            data.angular.z = error * self.__servoKp * math.pi / 180.0
        
        self.__cmdVel.publish(data)

if __name__ == '__main__':
    rospy.init_node('wall_trace')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    rate = rospy.Rate(20)
    wallTrace = WallTrace()
    while not rospy.is_shutdown():
        wallTrace.Run()
        rate.sleep()
