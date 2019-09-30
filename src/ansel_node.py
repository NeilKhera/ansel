#!/usr/bin/env python
import cv2
import rospy
import dynamic_reconfigure.client
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from Ansel.srv import Ansel, AnselResponse

class ansel:

    def __init__(self):
        self.bridge = CvBridge()
        self.client = dynamic_reconfigure.client.Client("camera")
        s = rospy.Service('ansel', Ansel, self.saveImage)
        print("Ready to save images...")

    def saveImage(self, req):
        self.client.update_configuration({'gain_auto': 'Off', 'exposure_auto': 'Continuous', 'exposure_auto_alg': 'Mean', 'exposure_auto_max': '500000'})
        rostopic = req.rostopic
        filepath = req.filepath
        rospy.loginfo("Request to save %s received. Saving at %s.", rostopic, filepath)

        for i in range(11):
            rospy.loginfo("Saving image %d...", i)
            self.client.update_configuration({'exposure_auto_target': (i * 10)})
            rospy.sleep(5)

            try:
                msg = rospy.wait_for_message(rostopic, Image, 5.0)
                sec = msg.header.stamp.secs
                nsec = msg.header.stamp.nsecs
                filename = "img_" + str(i) + "_" + str(sec) + "_" + str(nsec) + ".png"

                try:
                    image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
                except CvBridgeError:
                    return AnselResponse("ERROR! Writing image file failed!")

                cv2.imwrite(filepath + '/' + filename, image)
                rospy.loginfo("%s saved succesfully.", filename)
            except rospy.exceptions.ROSException:
                return AnselResponse("Error! No messages received in 5 seconds. Timeout!")
        
        return AnselResponse("Saved 10 files at" + filepath + ".")

def init():
    rf = ansel()
    rospy.init_node('ansel', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    init()
