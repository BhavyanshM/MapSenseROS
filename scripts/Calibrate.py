import cv2
import numpy as np
import rospy
import os
import glob
import sys
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

# CHECKERBOARD = (6,9)
#
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
#
# calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
# objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
# objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
# _img_shape = None
# objpoints = [] # 3d point in real world space
# imgpoints = [] # 2d points in image plane.
# images = glob.glob('*.jpg')
#
#
#
# for fname in images:
#     img = cv2.imread(fname)
#     if _img_shape == None:
#         _img_shape = img.shape[:2]
#     else:
#         assert _img_shape == img.shape[:2], "All images must share the same size."
#     gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#     # Find the chess board corners
#     ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
#     # If found, add object points, image points (after refining them)
#     if ret == True:
#         objpoints.append(objp)
#         cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
#         imgpoints.append(corners)
# N_OK = len(objpoints)
# K = np.zeros((3, 3))
# D = np.zeros((4, 1))
# rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
# tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
# rms, _, _, _, _ = \
#     cv2.fisheye.calibrate(
#         objpoints,
#         imgpoints,
#         gray.shape[::-1],
#         K,
#         D,
#         rvecs,
#         tvecs,
#         calibration_flags,
#         (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
#     )
# print("Found " + str(N_OK) + " valid images for calibration")
# print("DIM=" + str(_img_shape[::-1]))
# print("K=np.array(" + str(K.tolist()) + ")")
# print("D=np.array(" + str(D.tolist()) + ")")


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_color", Image,self.callback)

        self.processor = rospy.Timer(rospy.Duration(0.032), self.processingCallback)
        self.image = None
        self.imageSet = False

        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((6*7,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.

    def callback(self,data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.imageSet = True

    def capture_pattern(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
        if ret == True:
            self.objpoints.append(self.objp)
            corners2 = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), self.criteria)
            self.imgpoints.append(corners)
            # Draw and display the corners
            cv2.drawChessboardCorners(image, (8,6), corners2, ret)

    def processingCallback(self, timer):
        if self.imageSet:
            image = self.image

            self.capture_pattern(image)

            cv2.imshow("Final", image)
            code = cv2.waitKeyEx(1)

            if code == 113:
                rospy.signal_shutdown("Done")
                exit()

            self.imageSet = False

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