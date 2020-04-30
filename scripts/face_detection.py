#!/usr/bin/env python
#
# =======================================================================
#   @file   face_detection.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import rospy
import cv2
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge, CvBridgeError
from face_to_face import FaceToFace


class FaceDetection():

    __slots__ = (
        '_pubFace', '_cvBridge', '_imageOrg', '_scaleFactor', '_minNeighbors', '_cascade',
        '_subImage')

    def __init__(self):
        self._subImage = rospy.Subscriber("/cv_camera/image_raw", Image, self.GetImage)
        self._pubFace = rospy.Publisher("face", Image, queue_size=1)
        self._cvBridge = CvBridge()
        self._imageOrg = None

        self._scaleFactor = rospy.get_param("/vision_control/scale_factor", 1.3)
        self._minNeighbors = rospy.get_param("/vision_control/min_neighbors", 2)

        classifier = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
        self._cascade = cv2.CascadeClassifier(classifier)

    def GetImage(self, img):
        try:
            self._imageOrg = self._cvBridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def Monitor(self, rect, org):
        if rect is not None:
            cv2.rectangle(org, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 255), 4)

        self._pubFace.publish(self._cvBridge.cv2_to_imgmsg(org, "bgr8"))

    def DetectFace(self):
        if self._imageOrg is None:
            return None, None

        org = self._imageOrg

        gimg = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
        face = self._cascade.detectMultiScale(
            gimg, self._scaleFactor, self._minNeighbors, cv2.CASCADE_FIND_BIGGEST_OBJECT)

        if len(face) == 0:
            self.Monitor(None, org)
            return None, org

        r = face[0]
        self.Monitor(r, org)

        return r, org

    def Control(self):
        r, image = self.DetectFace()
        if r is None:
            return 0.0

        wid = image.shape[1] / 2
        xPosRate = (r[0] + r[2] / 2 - wid) * 1.0 / wid
        return xPosRate


if __name__ == "__main__":
    rospy.init_node("face_detection")
    rospy.wait_for_service('/motor_on')
    rospy.wait_for_service('/motor_off')
    rospy.on_shutdown(rospy.ServiceProxy('motor_off', Trigger).call)
    rospy.ServiceProxy('motor_on', Trigger).call()

    fd = FaceDetection()

    rate = rospy.Rate(15.0)
    ftf = FaceToFace()
    while not rospy.is_shutdown():
        ftf.Rotate(fd.Control())
        rate.sleep()
