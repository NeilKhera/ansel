#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rostopic2file.srv import Rostopic2File, Rostopic2FileResponse

class rostopic2file:

    def __init__(self):
        self.bridge = CvBridge()
        s = rospy.Service('rostopic2file', Rostopic2File, self.saveImage)
        print "Ready to save images..."

    def saveImage(self, req):
        rostopic = req.rostopic
        filepath = req.filepath

        try:
            msg = rospy.wait_for_message(rostopic, Image, 5.0)
            sec = msg.header.stamp.secs
            nsec = msg.header.stamp.nsecs

            filename = "img_" + str(sec) + "_" + str(nsec) + ".png"

            try:
                image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
            except CvBridgeError:
                return Rostopic2FileResponse("ERROR! Writing image file failed!")

            cv2.imwrite(filepath + filename, image)
            return Rostopic2FileResponse("Saved " + filename)
        except rospy.exceptions.ROSException:
            return Rostopic2FileResponse("Error! No messages received in 5 seconds. Timeout!")

def init():
    rf = rostopic2file()
    rospy.init_node('rostopic2file', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    init()
