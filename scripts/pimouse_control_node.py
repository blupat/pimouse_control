#!/usr/bin/env python

from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from wall_around import *
from wall_trace import *
from face_to_face import *
from face_detection import *
from pimouse_control.srv import PiMouseCmd, PiMouseCmdRequest, PiMouseCmdResponse

class PiMouseControl(object):
    __slots__ = ('__cmdVel', '__srvCmd', '__srvClientOn', '__srvClientOff', '__wallAround', \
                 '__wallTrace', '__faceToFace', '__faceDetection', '__isOn', '__isRun', \
                 '__on', '__run', '__face', '__forward', '__rotation')

    def __init__(self):
        self.__isOn = False
        self.__isRun = False
        self.__on = False
        self.__run = False
        self.__face = False
        self.__forward = 0.0
        self.__rotation = 0.0
        self.__cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.__srvCmd = rospy.Service('pimouse_cmd', PiMouseCmd, self.CommandCallback)
        self.__srvClientOn = rospy.ServiceProxy('motor_on', Trigger)
        self.__srvClientOff = rospy.ServiceProxy('motor_off', Trigger)
        rospy.on_shutdown(self.__srvClientOff.call)
        self.__wallAround = WallAround()
        self.__wallTrace = WallTrace()
        self.__faceToFace = FaceToFace()
        self.__faceDetection = FaceDetection()

    def CommandCallback(self, req):
        if req.on:
            self.__on = True
            if req.run:
                self.__run = True
                self.__face = False
                self.__forward = 0.0
                self.__rotation = 0.0
            elif req.face:
                self.__run = False
                self.__face = True
                self.__forward = 0.0
                self.__rotation = 0.0
            else:
                self.__run = False
                self.__face = False
                self.__forward = req.forward
                self.__rotation = req.rotation
        else:
            self.__on = False
            self.__run = False
            self.__face = False
            self.__forward = 0.0
            self.__rotation = 0.0
            self.__srvClientOff.call()
        res = PiMouseCmdResponse()
        res.isOk = True
        return res

    def Run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            try:
                xPosRate = self.__faceDetection.Control()
                if self.__on:
                    if not self.__isOn:
                        self.__srvClientOn.call()
                        self.__isOn = True
                    if self.__run:
                        if not self.__isRun:
                            self.__wallAround.Start()
                            self.__isRun = True
                        else:
                            self.__wallAround.Run()
                    elif self.__face:
                        self.__faceToFace.Rotate(xPosRate)
                        self.__isRun = False
                    else:
                        m = Twist()
                        m.linear.x = self.__forward
                        m.angular.z = self.__rotation
                        self.__cmdVel.publish(m)
                        self.__isRun = False
                else:
                    if self.__isOn:
                        m = Twist()
                        m.linear.x = 0.0
                        m.angular.z = 0.0
                        self.__cmdVel.publish(m)
                        self.__srvClientOff.call()
                        self.__isOn = False
                        self.__isRun = False
            except Exception as e:
                print(e)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pimouse_control_node')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    control = PiMouseControl()
    control.Run()
