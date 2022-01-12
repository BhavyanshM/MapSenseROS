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

import yaml
from sensor_msgs.msg import CameraInfo

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by
    rosrun camera_calibration cameracalibrator.py) into a
    sensor_msgs/CameraInfo msg.

    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.compressed = True

        self.depthPub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=2)
        self.depthInfo = rospy.Publisher('/camera/aligned_depth_to_color/camera_info', CameraInfo, queue_size=2)

        if self.compressed is True:
            print("Subscriber Added: ", "/zed/zed_node/left/image_rect_color/compressed")
            print("Subscriber Added: ", "/zed/zed_node/right/image_rect_color/compressed")
            self.leftImage_sub = rospy.Subscriber("/zed/zed_node/left/image_rect_color/compressed", CompressedImage,self.leftCallback)
            self.rightImage_sub = rospy.Subscriber("/zed/zed_node/right/image_rect_color/compressed", CompressedImage,self.rightCallback)
        else:
            print("Subscriber Added: ", "/zed/color/left/image_raw")
            print("Subscriber Added: ", "/zed/color/right/image_raw")
            self.leftImage_sub = rospy.Subscriber("/zed/color/left/image_raw", Image,self.leftCallback)
            self.rightImage_sub = rospy.Subscriber("/zed/color/right/image_raw", Image,self.rightCallback)

        self.processor = rospy.Timer(rospy.Duration(0.01), self.processingCallback)
        self.leftImage = None
        self.rightImage = None
        self.disparity = None
        self.leftSet = False
        self.rightSet = False

        # stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        #Set disparity parameters
        #Note: disparity range is tuned according to specific parameters obtained through trial and error.
        self.win_size = 5
        self.min_disp = -1
        self.max_disp = 63 #min_disp * 9
        self.num_disp = self.max_disp - self.min_disp # Needs to be divisible by 16
        self.stereo = cv2.StereoSGBM_create(minDisparity= self.min_disp,
                                       numDisparities = self.num_disp,
                                       blockSize = 5,
                                       uniquenessRatio = 5,
                                       speckleWindowSize = 5,
                                       speckleRange = 5,
                                       disp12MaxDiff = 1,
                                       P1 = 8*3*self.win_size**2,#8*3*win_size**2,
                                       P2 =32*3*self.win_size**2) #32*3*win_size**2)

    def leftCallback(self,data):
        try:
            if self.compressed is True:
                np_arr = np.frombuffer(data.data, np.uint8)
                input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                input_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.leftImage = input_image
            self.leftSet = True
        except CvBridgeError as e:
            print(e)

        # self.leftImage = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

    def rightCallback(self,data):
        try:
            if self.compressed is True:
                np_arr = np.frombuffer(data.data, np.uint8)
                input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                input_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            self.rightImage = input_image
            self.rightSet = True
            # print(data.header)
        except CvBridgeError as e:
            print(e)

        # self.leftImage = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

    def processingCallback(self, timer):
        if self.rightSet and self.leftSet:

            leftImage = cv2.resize(cv2.cvtColor(self.leftImage, cv2.COLOR_BGR2GRAY), (int(self.leftImage.shape[1]/2), int(self.leftImage.shape[0]/2)))
            rightImage = cv2.resize(cv2.cvtColor(self.rightImage, cv2.COLOR_BGR2GRAY), (int(self.rightImage.shape[1]/2), int(self.rightImage.shape[0]/2)))

            self.disparity = self.stereo.compute(leftImage, rightImage)

            depthMap = 65536 / np.abs(self.disparity)
            depth = np.array(depthMap, dtype=np.uint16)

            norm_image = cv2.normalize(depth, None, alpha = 0, beta = 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

            final = np.hstack([leftImage, rightImage])
            cv2.imshow("Final", depthMap)
            cv2.imshow("Color", final)
            cv2.waitKey(1)



            depthMsg = self.bridge.cv2_to_imgmsg(depth, "16UC1")
            self.depthPub.publish(depthMsg)

            # cv2.imwrite("LeftImage.jpg", self.leftImage)
            # exit()

            self.rightSet = False
            self.leftSet = False


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