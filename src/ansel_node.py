#!/usr/bin/env python
import datetime
import numpy as np
import cv2
import rospy
import dynamic_reconfigure.client
from fractions import Fraction
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ansel.srv import Ansel, AnselResponse

class ansel:

    def __init__(self):
        self.bridge = CvBridge()
        self.client = dynamic_reconfigure.client.Client("camera")
        s = rospy.Service('ansel', Ansel, self.saveImage)
        print("Ready to save images...")

    def saveImage(self, req):
        self.client.update_configuration({'gain_auto': 'Off', 'exposure_auto': 'Continuous', 'exposure_auto_alg': 'Mean', 'exposure_auto_max': '60000000'})
        rostopic = req.rostopic
        filepath = req.filepath
        rospy.loginfo("Request to save %s received. Saving at %s.", rostopic, filepath)

        if ((req.image_count - 1) * req.step_size + req.base_grey > 100):
            return AnselResponse("ERROR! Invalid Parameters!")

        image_arr = []
        time_arr = []
        old_grey = 0

        for i in range(req.image_count):
            rospy.loginfo("Saving image %d...", i)
            new_grey = ((i * req.step_size) + req.base_grey)
            self.client.update_configuration({'exposure_auto_target': new_grey})
            rospy.sleep(3)

            try:
                msg = rospy.wait_for_message(rostopic, Image, 5.0)
                sec = msg.header.stamp.secs
                nsec = msg.header.stamp.nsecs
                filename = "img_" + str(i) + "_" + str(sec) + "_" + str(nsec) + ".png"

                if i == 0:
                    time = 1.0
                else:
                    base = 2.0
                    exp = (float) (new_grey - old_grey) / 10
                    time = time * (base ** exp)
                time_arr.append(time)
                old_grey = new_grey

                try:
                    image = self.bridge.imgmsg_to_cv2(msg, msg.encoding)
                    image_arr.append(image)
                except CvBridgeError:
                    return AnselResponse("ERROR! Writing image file failed!")

                cv2.imwrite(filepath + "/" + filename, image)
                rospy.loginfo("%s saved successfully.", filename)
            except rospy.exceptions.ROSException:
                return AnselResponse("Error! No messages received in 5 seconds. Timeout!")
        
        if (req.hdr):
            time_arr = np.asarray(time_arr, dtype=np.float32)

            rospy.loginfo("Saving HDR image...")
            alignMTB = cv2.createAlignMTB()
            alignMTB.process(image_arr, image_arr)

            calibrateDebevec = cv2.createCalibrateDebevec()
            responseDebevec = calibrateDebevec.process(image_arr, time_arr)

            mergeDebevec = cv2.createMergeDebevec()
            hdrDebevec = mergeDebevec.process(image_arr, time_arr, responseDebevec)
            
            DT = datetime.datetime.now()
            cv2.imwrite(filepath + "/" + "img_" + str(DT.second) + "_" + str(DT.microsecond) + ".hdr", hdrDebevec)
            rospy.loginfo("HDR image saved successfully.")

            return AnselResponse("Saved " + str(req.image_count) + " images and an HDR image at " + filepath + ".")

        return AnselResponse("Saved " + str(req.image_count) + " images at " + filepath + ".")

def init():
    rf = ansel()
    rospy.init_node('ansel', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    init()
