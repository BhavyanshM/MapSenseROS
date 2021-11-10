#!/usr/bin/env Python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.leftImage_sub = rospy.Subscriber("/zed/zed_node/left/image_rect_color/compressed", CompressedImage,self.leftCallback)
        self.rightImage_sub = rospy.Subscriber("/zed/zed_node/right/image_rect_color/compressed", CompressedImage,self.rightCallback)
        self.processor = rospy.Timer(rospy.Duration(0.01), self.processingCallback)
        self.leftImage = None
        self.rightImage = None
        self.disparity = None
        self.leftSet = False
        self.rightSet = False

    def leftCallback(self,data):
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # print(data.header)
            np_arr = np.frombuffer(data.data, np.uint8)
            input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.leftImage = input_image
            self.leftSet = True
        except CvBridgeError as e:
            print(e)

        # self.leftImage = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

    def rightCallback(self,data):
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            np_arr = np.frombuffer(data.data, np.uint8)
            input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.rightImage = input_image
            self.rightSet = True
            # print(data.header)
        except CvBridgeError as e:
            print(e)

        # self.leftImage = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

    def processingCallback(self, timer):
        if self.rightSet and self.leftSet:
            print("Stereo Pair Ready")

            # stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            #Set disparity parameters
            #Note: disparity range is tuned according to specific parameters obtained through trial and error.
            win_size = 5
            min_disp = -1
            max_disp = 63 #min_disp * 9
            num_disp = max_disp - min_disp # Needs to be divisible by 16
            stereo = cv2.StereoSGBM_create(minDisparity= min_disp,
                                           numDisparities = num_disp,
                                           blockSize = 5,
                                           uniquenessRatio = 5,
                                           speckleWindowSize = 5,
                                           speckleRange = 5,
                                           disp12MaxDiff = 1,
                                           P1 = 8*3*win_size**2,#8*3*win_size**2,
                                           P2 =32*3*win_size**2) #32*3*win_size**2)

            leftImage = cv2.resize(cv2.cvtColor(self.leftImage, cv2.COLOR_BGR2GRAY), (int(self.leftImage.shape[1]/2), int(self.leftImage.shape[0]/2)))
            rightImage = cv2.resize(cv2.cvtColor(self.rightImage, cv2.COLOR_BGR2GRAY), (int(self.rightImage.shape[1]/2), int(self.rightImage.shape[0]/2)))

            self.disparity = stereo.compute(leftImage, rightImage)

            norm_image = cv2.normalize(self.disparity, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

            final = np.hstack([leftImage, rightImage])
            cv2.imshow("Final", norm_image)
            cv2.imshow("Color", final)
            cv2.waitKey(1)


            cv2.imwrite("LeftImage.jpg", self.leftImage)

            exit()

            self.rightSet = False
            self.leftSet = False

    # def calculateStereo(self, left, right):


def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)