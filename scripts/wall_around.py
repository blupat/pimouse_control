#!/usr/bin/env python
import rospy, copy, math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues

class WallAround():
    __slots__ = ('__cmdVel', '__accel', '__maxSpeed', '__minSpeed', '__servoTarget', '__servoKp', '__servoKd', \
                 '__sensorOffThreshold', '__wallGain', '__wallThreshold', '__linearSpeed', '__sensorValues', \
                 '__linearSpeed', '__angularSpeed', '__previousError', '__previousTime')

    def __init__(self):
        self.__cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.__accel = rospy.get_param("/run_corridor/acceleration", 0.01)
        self.__maxSpeed = rospy.get_param("/run_corridor/max_speed", 0.3)
        self.__minSpeed = rospy.get_param("/run_corridor/min_speed", 0.0)
        self.__servoTarget = rospy.get_param("/run_corridor/servo_target", 40)
        self.__servoKp = rospy.get_param("/run_corridor/servo_kp", 1.0)
        self.__servoKd = rospy.get_param("/run_corridor/servo_kd", 0.01)
        self.__servoOffThreshold = rospy.get_param("/run_corridor/servo_off_threshold", 10)
        self.__wallGain = rospy.get_param("/run_corridor/wall_gain", 0.75)
        self.__wallThreshold = rospy.get_param("/run_corridor/wall_threshold", 20)
        self.__forwardThreshold = rospy.get_param("/run_corridor/forward_threshold", 30)
        self.__rightThreshold = rospy.get_param("/run_corridor/right_threshold", 40)
        self.__leftThreshold = rospy.get_param("/run_corridor/left_threshold", 80)
        
        self.__sensorValues = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.Callback)

    def Callback(self, messages):
        self.__sensorValues = messages

    def WallFront(self, ls):
        return (ls.left_forward > self.__wallThreshold) or (ls.right_forward > self.__wallThreshold)
    
    def TooRight(self, ls):
        return (ls.right_side > self.__wallThreshold)
    
    def TooLeft(self, ls):
        return (ls.left_side > self.__wallThreshold)

    def Start(self):
        self.__previousError = 0.0
        self.__previousTime = rospy.get_time()
        self.__linearSpeed = self.__minSpeed
        self.__angularSpeed = 0.0

    def Run(self):
        data = Twist()
        ls = self.__sensorValues
        error = 0.0
        nowTime = rospy.get_time()
        if self.WallFront(ls):
            self.__linearSpeed = 0.0
            if self.TooRight(ls):
                self.__angularSpeed = math.pi * self.__wallGain
            else:
                self.__angularSpeed = - math.pi * self.__wallGain
        elif self.TooRight(ls):
            self.__linearSpeed -= self.__accel
            self.__angularSpeed = math.pi * self.__wallGain
        elif self.TooLeft(ls):
            self.__linearSpeed -= self.__accel
            self.__angularSpeed = - math.pi * self.__wallGain
        else:
            if ls.sum_forward > self.__forwardThreshold:
                self.__linearSpeed -= self.__accel
            else:
                self.__linearSpeed += self.__accel
            if ls.left_side < self.__servoOffThreshold:
                self.__angularSpeed = 0.0
            else:
                error = self.__servoTarget - ls.left_side
                self.__angularSpeed = error * self.__servoKp * math.pi / 180.0
                deltaError = error - self.__previousError
                deltaTime = nowTime - self.__previousTime
                if deltaTime > 0.0000001:
                    self.__angularSpeed += deltaError / deltaTime * self.__servoKd * math.pi / 180.0
        data.linear.x = self.__linearSpeed
        data.angular.z = self.__angularSpeed

        self.__previousError = error
        self.__previousTime = nowTime
        if data.linear.x <= self.__minSpeed:
            data.linear.x = self.__minSpeed
        elif data.linear.x >= self.__maxSpeed:
            data.linear.x = self.__maxSpeed

        self.__cmdVel.publish(data)

if __name__ == '__main__':
    rospy.init_node('wall_around')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
    rospy.ServiceProxy('/motor_on', Trigger).call()
    wallAround = WallAround()
    isRun = False
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        if not isRun:
            wallAround.Start()
            isRun = True
        else:
            wallAround.Run()
        rate.sleep()