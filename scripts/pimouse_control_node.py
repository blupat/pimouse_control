#!/usr/bin/env python
#
# =======================================================================
#   @file   pimouse_control_node.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from wall_around import WallAround
from face_to_face import FaceToFace
from face_detection import FaceDetection
from pimouse_control.srv import PiMouseCmd, PiMouseCmdRequest, PiMouseCmdResponse


class PiMouseControl(object):

    __slots__ = (
        '_srvCmd', '_srvClientOn', '_srvClientOff', '_wallAround',
        '_faceToFace', '_faceDetection', '_isOn', '_isRun',
        '_on', '_run', '_face', '_forward', '_rotation')

    def __init__(self):
        self._isOn = False
        self._isRun = False
        self._on = False
        self._run = False
        self._face = False
        self._forward = 0.0
        self._rotation = 0.0
        self._srvCmd = rospy.Service('pimouse_cmd', PiMouseCmd, self.CommandCallback)
        self._srvClientOn = rospy.ServiceProxy('motor_on', Trigger)
        self._srvClientOff = rospy.ServiceProxy('motor_off', Trigger)
        rospy.on_shutdown(self._srvClientOff.call)
        self._wallAround = WallAround()
        self._faceToFace = FaceToFace()
        self._faceDetection = FaceDetection()

    def CommandCallback(self, req):
        if req.on:
            self._on = True
            if req.run:
                self._run = True
                self._face = False
                self._forward = 0.0
                self._rotation = 0.0
            elif req.face:
                self._run = False
                self._face = True
                self._forward = 0.0
                self._rotation = 0.0
            else:
                self._run = False
                self._face = False
                self._forward = req.forward
                self._rotation = req.rotation
        else:
            self._on = False
            self._run = False
            self._face = False
            self._forward = 0.0
            self._rotation = 0.0
            self._srvClientOff.call()
        res = PiMouseCmdResponse()
        res.isOk = True
        return res

    def Run(self):
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            try:
                xPosRate = self._faceDetection.Control()
                if self._on:
                    if not self._isOn:
                        self._srvClientOn.call()
                        self._isOn = True
                    if self._run:
                        if not self._isRun:
                            self._wallAround.Start()
                            self._isRun = True
                        else:
                            self._wallAround.Run()
                    elif self._face:
                        self._faceToFace.Rotate(xPosRate)
                        self._isRun = False
                    else:
                        self._wallAround.SetVelocity(self._forward, self._rotation)
                        self._isRun = False
                else:
                    if self._isOn:
                        self._wallAround.SetVelocity(0.0, 0.0)
                        self._srvClientOff.call()
                        self._isOn = False
                        self._isRun = False
            except Exception as e:
                print(e)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pimouse_control_node')
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    control = PiMouseControl()
    control.Run()
