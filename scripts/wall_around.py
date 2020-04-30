#!/usr/bin/env python
#
# =======================================================================
#   @file   wall_around.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rospy
import copy
import math
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse
from pimouse_ros.msg import LightSensorValues
from pimouse_control.msg import RunData


class DistanceValues():

    __slots__ = (
        'right_forward', 'right_side', 'left_side', 'left_forward'
    )

    def __init__(self, sensorValues):
        if sensorValues.right_forward > 0:
            self.right_forward = math.sqrt(sensorValues.right_forward)
        else:
            self.right_forward = 0.0
        if sensorValues.right_side > 0:
            self.right_side = math.sqrt(sensorValues.right_side)
        else:
            self.right_side = 0.0
        if sensorValues.left_side > 0:
            self.left_side = math.sqrt(sensorValues.left_side)
        else:
            self.left_side = 0.0
        if sensorValues.left_forward > 0:
            self.left_forward = math.sqrt(sensorValues.left_forward)
        else:
            self.left_forward = 0.0


class WallAround():

    __slots__ = (
        '_cmdVel', '_accel', '_decel', '_maxSpeed', '_minSpeed', '_servoTarget',
        '_servoKp', '_servoKd', '_leftSideBuffer', '_rightSideBuffer', '_bufferIndex',
        '_servoOffThreshold', '_wallGain', '_wallThreshold',
        '_distanceValues', '_leftAverage', '_rightAverage',
        '_linearSpeed', '_angularSpeed', '_previousError', '_previousTime',
        '_startTime', '_x', '_y', '_th', '_pubRunData', '_isServoOn', '_isTurnRight',
        '_rightThreshold', '_leftThreshold')

    def __init__(self):
        self._cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._pubRunData = rospy.Publisher('/run_data', RunData, queue_size=1)

        self._accel = rospy.get_param("/run_corridor/acceleration", 0.01)
        self._decel = rospy.get_param("/run_corridor/deceleration", 0.05)
        self._maxSpeed = rospy.get_param("/run_corridor/max_speed", 0.3)
        self._minSpeed = rospy.get_param("/run_corridor/min_speed", 0.0)
        self._servoTarget = rospy.get_param("/run_corridor/servo_target", 12.0)
        self._servoKp = rospy.get_param("/run_corridor/servo_kp", 10.0)
        self._servoKd = rospy.get_param("/run_corridor/servo_kd", 0.2)
        self._servoOffThreshold = rospy.get_param("/run_corridor/servo_off_threshold", 3.0)
        self._wallGain = rospy.get_param("/run_corridor/wall_gain", 1.0)
        self._wallThreshold = rospy.get_param("/run_corridor/wall_threshold", 16.0)
        self._rightThreshold = rospy.get_param("/run_corridor/right_threshold", 20.0)
        self._leftThreshold = rospy.get_param("/run_corridor/left_threshold", 20.0)

        self._distanceValues = DistanceValues(LightSensorValues())
        rospy.Subscriber('/lightsensors', LightSensorValues, self.Callback)

    def Callback(self, messages):
        self._distanceValues = DistanceValues(messages)
        self._leftSideBuffer[self._bufferIndex] = self._distanceValues.left_side
        self._rightSideBuffer[self._bufferIndex] = self._distanceValues.right_side
        self._bufferIndex += 1
        if self._bufferIndex >= 3:
            self._bufferIndex = 0
        self._leftAverage = sum(self._leftSideBuffer) / len(self._leftSideBuffer)
        self._rightAverage = sum(self._rightSideBuffer) / len(self._rightSideBuffer)

    def WallFront(self, dv):
        return (dv.left_forward > self._wallThreshold) or (dv.right_forward > self._wallThreshold)

    def TooRight(self, dv):
        return (dv.right_side > self._rightThreshold)

    def TooLeft(self, dv):
        return (dv.left_side > self._leftThreshold)

    def Start(self):
        self._startTime = rospy.get_time()
        self._previousError = 0.0
        self._previousTime = self._startTime
        self._linearSpeed = 0.0
        self._angularSpeed = 0.0
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._isServoOn = False
        self._isTurnRight = False
        self._leftSideBuffer = [0.0, 0.0, 0.0]
        self._rightSideBuffer = [0.0, 0.0, 0.0]
        self._bufferIndex = 0

    def Run(self):
        dv = self._distanceValues
        leftAverage = self._leftAverage
        rightAverage = self._rightAverage

        error = 0.0
        deltaError = 0.0
        nowTime = rospy.get_time()
        elapsedTime = nowTime - self._startTime
        deltaTime = nowTime - self._previousTime
        isWallFront = False

        if self.WallFront(dv):
            self._linearSpeed = 0.0
            if self._isServoOn:
                if rightAverage > leftAverage:
                    self._isTurnRight = False
                else:
                    self._isTurnRight = True
            if self._isTurnRight:
                self._angularSpeed = - math.pi * self._wallGain
            else:
                self._angularSpeed = math.pi * self._wallGain
            self._isServoOn = False
            isWallFront = True
        else:
            if self.TooLeft(dv) or self.TooRight(dv):
                self._linearSpeed -= self._decel
                if self._linearSpeed < self._minSpeed:
                    self._linearSpeed = self._minSpeed
            else:
                self._linearSpeed += self._accel

            if ((dv.left_side < self._servoOffThreshold)
                    and (dv.right_side < self._servoOffThreshold)):
                self._angularSpeed = 0.0
                self._isServoOn = False
                self._isTurnRight = False
            elif rightAverage > leftAverage:
                error = dv.right_side - self._servoTarget
                self._angularSpeed = error * self._servoKp * math.pi / 180.0
                if self._isServoOn and self._isTurnRight:
                    deltaError = error - self._previousError
                    self._angularSpeed += deltaError / deltaTime * self._servoKd * math.pi / 180.0
                self._isServoOn = True
                self._isTurnRight = True
            else:
                error = self._servoTarget - dv.left_side
                self._angularSpeed = error * self._servoKp * math.pi / 180.0
                if self._isServoOn and (not self._isTurnRight):
                    deltaError = error - self._previousError
                    self._angularSpeed += deltaError / deltaTime * self._servoKd * math.pi / 180.0
                self._isServoOn = True
                self._isTurnRight = False

        if self._linearSpeed < 0.0:
            self._linearSpeed = 0.0
        elif self._linearSpeed > self._maxSpeed:
            self._linearSpeed = self._maxSpeed

        data = Twist()
        data.linear.x = self._linearSpeed
        data.angular.z = self._angularSpeed
        self._cmdVel.publish(data)

        self._x += self._linearSpeed * math.cos(self._th) * deltaTime
        self._y += self._linearSpeed * math.sin(self._th) * deltaTime
        self._th += self._angularSpeed * deltaTime
        self._previousError = error
        self._previousTime = nowTime

        runData = RunData()
        runData.elapsedTime = elapsedTime
        runData.x = self._x
        runData.y = self._y
        runData.th = self._th
        runData.linear = self._linearSpeed
        runData.angular = self._angularSpeed
        runData.error = error
        runData.deltaError = deltaError
        runData.leftSide = dv.left_side
        runData.leftForward = dv.left_forward
        runData.rightForward = dv.right_forward
        runData.rightSide = dv.right_side
        runData.isServoOn = self._isServoOn
        runData.isTurnRight = self._isTurnRight
        runData.isWallFront = isWallFront

        self._pubRunData.publish(runData)

    def SetVelocity(self, linear, angular):
        data = Twist()
        dv = DistanceValues(self._sensorValues)
        if self.WallFront(dv) and (linear > 0.0):
            data.linear.x = 0.0
        else:
            data.linear.x = linear
        data.angular.z = angular
        self._cmdVel.publish(data)


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
