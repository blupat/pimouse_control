#!/usr/bin/env python
import rospy, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from face_to_face import *

class FaceDetection():
    __slots__ = ('__pubFace', '__pubPos', '__cvBridge', '__imageOrg', '__scaleFactor', '__minNeighbors', '__cascade')

    def __init__(self):
        sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.GetImage)
        self.__pubFace = rospy.Publisher("face", Image, queue_size = 1)
        self.__cvBridge = CvBridge()
        self.__imageOrg = None
        
        self.__scaleFactor = rospy.get_param("/vision_control/scale_factor", 1.3)
        self.__minNeighbors = rospy.get_param("/vision_control/min_neighbors", 2)
        
        classifier = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
        self.__cascade = cv2.CascadeClassifier(classifier)
    
    def GetImage(self, img):
        try:
            self.__imageOrg = self.__cvBridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def Monitor(self, rect, org):
        if rect is not None:
            cv2.rectangle(org, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 255), 4)
        
        self.__pubFace.publish(self.__cvBridge.cv2_to_imgmsg(org, "bgr8"))
    
    def DetectFace(self):
        if self.__imageOrg is None:
            return None, None
        
        org = self.__imageOrg
        
        gimg = cv2.cvtColor(org, cv2.COLOR_BGR2GRAY)
        face = self.__cascade.detectMultiScale(gimg, self.__scaleFactor, self.__minNeighbors, cv2.CASCADE_FIND_BIGGEST_OBJECT)
        
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
