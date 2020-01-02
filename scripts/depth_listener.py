import sys, time# numpy and scipy
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class image_feature:

    def __init__(self):
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/depth_image",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print( "subscribed to /camera/image/compressed")


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print( 'received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        print(image_np.shape)
        cv2.namedWindow("DepthCamera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("DepthCamera", 1024, 758)
        cv2.imshow('DepthCamera', image_np)
        cv2.waitKey(2)



def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down ROS depth subscriber")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)