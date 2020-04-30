#!/usr/bin/env python
import rospy, copy, math
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
        '__cmdVel', '__accel', '__decel', '__maxSpeed', '__minSpeed', '__servoTarget',
        '__servoKp', '__servoKd', '__leftSideBuffer', '__rightSideBuffer', '__bufferIndex',
        '__servoOffThreshold', '__wallGain', '__wallThreshold', '__linearSpeed',
        '__sensorValues', '__linearSpeed', '__angularSpeed', '__previousError', '__previousTime',
        '__startTime', '__x', '__y', '__th', '__pubRunData', '__isServoOn', '__isTurnRight',
        '__rightThreshold', '__leftThreshold')

    def __init__(self):
        self.__cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.__pubRunData = rospy.Publisher('/run_data', RunData, queue_size=1)

        self.__accel = rospy.get_param("/run_corridor/acceleration", 0.01)
        self.__decel = rospy.get_param("/run_corridor/deceleration", 0.05)
        self.__maxSpeed = rospy.get_param("/run_corridor/max_speed", 0.3)
        self.__minSpeed = rospy.get_param("/run_corridor/min_speed", 0.0)
        self.__servoTarget = rospy.get_param("/run_corridor/servo_target", 12.0)
        self.__servoKp = rospy.get_param("/run_corridor/servo_kp", 10.0)
        self.__servoKd = rospy.get_param("/run_corridor/servo_kd", 0.2)
        self.__servoOffThreshold = rospy.get_param("/run_corridor/servo_off_threshold", 3.0)
        self.__wallGain = rospy.get_param("/run_corridor/wall_gain", 1.0)
        self.__wallThreshold = rospy.get_param("/run_corridor/wall_threshold", 16.0)
        self.__rightThreshold = rospy.get_param("/run_corridor/right_threshold", 20.0)
        self.__leftThreshold = rospy.get_param("/run_corridor/left_threshold", 20.0)

        self.__sensorValues = LightSensorValues()
        rospy.Subscriber('/lightsensors', LightSensorValues, self.Callback)

    def Callback(self, messages):
        self.__sensorValues = messages

    def WallFront(self, dv):
        return (dv.left_forward > self.__wallThreshold) or (dv.right_forward > self.__wallThreshold)

    def TooRight(self, dv):
        return (dv.right_side > self.__rightThreshold)

    def TooLeft(self, dv):
        return (dv.left_side > self.__leftThreshold)

    def Start(self):
        self.__startTime = rospy.get_time()
        self.__previousError = 0.0
        self.__previousTime = self.__startTime
        self.__linearSpeed = 0.0
        self.__angularSpeed = 0.0
        self.__x = 0.0
        self.__y = 0.0
        self.__th = 0.0
        self.__isServoOn = False
        self.__isTurnRight = False
        self.__leftSideBuffer = [0.0, 0.0, 0.0]
        self.__rightSideBuffer = [0.0, 0.0, 0.0]
        self.__bufferIndex = 0

    def Run(self):
        dv = DistanceValues(self.__sensorValues)
        self.__leftSideBuffer[self.__bufferIndex] = dv.left_side
        self.__rightSideBuffer[self.__bufferIndex] = dv.right_side
        self.__bufferIndex += 1
        if self.__bufferIndex >= 3:
            self.__bufferIndex = 0

        error = 0.0
        deltaError = 0.0
        nowTime = rospy.get_time()
        elapsedTime = nowTime - self.__startTime
        deltaTime = nowTime - self.__previousTime
        leftAverage = sum(self.__leftSideBuffer) / len(self.__leftSideBuffer)
        rightAverage = sum(self.__rightSideBuffer) / len(self.__rightSideBuffer)
        isWallFront = False

        if self.WallFront(dv):
            self.__linearSpeed = 0.0
            if self.__isServoOn:
                if rightAverage > leftAverage:
                    self.__isTurnRight = False
                else:
                    self.__isTurnRight = True
            if self.__isTurnRight:
                self.__angularSpeed = - math.pi * self.__wallGain
            else:
                self.__angularSpeed = math.pi * self.__wallGain
            self.__isServoOn = False
            isWallFront = True
        else:
            if self.TooLeft(dv) or self.TooRight(dv):
                self.__linearSpeed -= self.__decel
                if self.__linearSpeed < self.__minSpeed:
                    self.__linearSpeed = self.__minSpeed
            else:
                self.__linearSpeed += self.__accel

            if (dv.left_side < self.__servoOffThreshold) and (dv.right_side < self.__servoOffThreshold):
                self.__angularSpeed = 0.0
                self.__isServoOn = False
                self.__isTurnRight = False
            elif rightAverage > leftAverage:
                error = dv.right_side - self.__servoTarget
                self.__angularSpeed = error * self.__servoKp * math.pi / 180.0
                if self.__isServoOn and self.__isTurnRight:
                    deltaError = error - self.__previousError
                    self.__angularSpeed += deltaError / deltaTime * self.__servoKd * math.pi / 180.0
                self.__isServoOn = True
                self.__isTurnRight = True
            else:
                error = self.__servoTarget - dv.left_side
                self.__angularSpeed = error * self.__servoKp * math.pi / 180.0
                if self.__isServoOn and (not self.__isTurnRight):
                    deltaError = error - self.__previousError
                    self.__angularSpeed += deltaError / deltaTime * self.__servoKd * math.pi / 180.0
                self.__isServoOn = True
                self.__isTurnRight = False

        if self.__linearSpeed < 0.0:
            self.__linearSpeed = 0.0
        elif self.__linearSpeed > self.__maxSpeed:
            self.__linearSpeed = self.__maxSpeed

        data = Twist()
        data.linear.x = self.__linearSpeed
        data.angular.z = self.__angularSpeed
        self.__cmdVel.publish(data)

        self.__x += self.__linearSpeed * math.cos(self.__th) * deltaTime
        self.__y += self.__linearSpeed * math.sin(self.__th) * deltaTime
        self.__th += self.__angularSpeed * deltaTime
        self.__previousError = error
        self.__previousTime = nowTime

        runData = RunData()
        runData.elapsedTime = elapsedTime
        runData.x = self.__x
        runData.y = self.__y
        runData.th = self.__th
        runData.linear = self.__linearSpeed
        runData.angular = self.__angularSpeed
        runData.error = error
        runData.deltaError = deltaError
        runData.leftSide = dv.left_side
        runData.leftForward = dv.left_forward
        runData.rightForward = dv.right_forward
        runData.rightSide = dv.right_side
        runData.isServoOn = self.__isServoOn
        runData.isTurnRight = self.__isTurnRight
        runData.isWallFront = isWallFront

        self.__pubRunData.publish(runData)

    def SetVelocity(self, linear, angular):
        data = Twist()
        dv = DistanceValues(self.__sensorValues)
        if self.WallFront(dv) and (linear > 0.0):
            data.linear.x = 0.0
        else:
            data.linear.x = linear
        data.angular.z = angular
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
