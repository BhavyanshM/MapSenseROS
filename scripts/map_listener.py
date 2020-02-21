import sys, time# numpy and scipy
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from map_sense.msg import PlanarRegion
import time
# from cv_bridge import CvBridge, CvBridgeError
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})


class image_feature:

    def __init__(self):
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/map/regions", CompressedImage, self.callback,  queue_size = 1)
       

    def callback(self, ros_data):
        global prev_time, fps, time_diff, disp 
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        # image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2RGBA)
        print(image_np[48,0,:], image_np.dtype)
        # print(imgOut1[24,32,:])
        # print(imgOut2[24,32,:])
        # print(image_np[0,0,2], imgOut[0,0,2], imgMed.dtype, imgOut.dtype)

def main(args):
    '''Initializes and cleanup ros node'''
    node = rospy.init_node('image_feature', anonymous=True)
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print( "Shutting down ROS depth subscriber")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)