import cv2


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
        self.image_sub = rospy.Subscriber("/camera/image_color", Image,self.callback)
        self.rectImage_pub = rospy.Publisher("/blackfly/rect/image_color/compressed", CompressedImage, queue_size=2)

        self.processor = rospy.Timer(rospy.Duration(0.01), self.processingCallback)
        self.image = None
        self.imageSet = False

        # D = [-0.17758659685785708, 0.01851769485901313, 0.0037698702321492337, -0.0021964409401091624, 0.0]
        K = np.array([499.3716197917922, 0.0, 1043.8826790137316, 0.0, 506.42956667285574, 572.2558510618412, 0.0, 0.0, 1.0]).reshape((3,3))
        K += np.array([555.639135420475, 0.0, 1013.2012178038589, 0.0, 556.1474774304701, 548.1923579820871, 0.0, 0.0, 1.0]).reshape((3,3))

        self.cameraMatrix = K / 2
        self.distCoeffs = np.array([-0.1304880574839372, 0.0343337720836711, 0, 0, 0.002347490605947351, 0.163868408051474, -0.02493286434834704, 0.01394671162254435])


    def callback(self,data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # np_arr = np.frombuffer(data.data, np.uint8)
        # input_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # self.image = input_image
        self.imageSet = True

    def processingCallback(self, timer):
        if self.imageSet:
            print("Stereo Pair Ready")

            image = cv2.resize(self.image, (2*int(self.image.shape[1]/2), 2*int(self.image.shape[0]/2)))
            image = cv2.undistort(image, self.cameraMatrix, self.distCoeffs)
            # image = cv2.resize(image, (640, 480))

            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
            self.rectImage_pub.publish(msg)


            cv2.imshow("Final", image)
            code = cv2.waitKeyEx(1)

            if code == 113:
                rospy.signal_shutdown("Done")
                exit()

            self.imageSet = False

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