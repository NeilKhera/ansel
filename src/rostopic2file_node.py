#!/usr/bin/env python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rostopic2file.srv import Rostopic2File, Rostopic2FileResponse

def saveImage(req):
    rostopic = req.rostopic
    filepath = req.filepath

    try:
        msg = rospy.wait_for_message(rostopic, Image, 5.0)
        sec = msg.header.stamp.sec
        nsec = msg.header.stamp.nsec

        filename = "img_" + sec + "_" + nsec + ".png"

        try:
            image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
        except CvBridgeError:
            return Rostopic2FileResponse("Writing image file failed!")

        cv2.imwrite(filepath + filename, image)
        return Rostopic2FileResponse("Saved " + filename)
    except rospy.exceptions.ROSException:
        return Rostopic2File("No messages received in 5 seconds. Timeout!")

def init():
    rospy.init_node('rostopic2file', anonymous=True)
    s = rospy.Service('rostopic2file', Rostopic2File, saveImage)
    print "Ready to save images..."
    rospy.spin()

if __name__ == '__main__':
    init()
