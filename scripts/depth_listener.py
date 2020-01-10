import sys, time# numpy and scipy
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
import pyopencl as cl
import time
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

context = cl.Context([cl.get_platforms()[0].get_devices()[0]])
queue = cl.CommandQueue(context)
program = cl.Program(context, open('kernel.cl').read()).build()
kernel = cl.Kernel(program, 'depthKernel')

prev_time = time.time()
fps = 0
time_diff = 0

class image_feature:

    def __init__(self):
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/depth_image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print( "subscribed to /camera/image/compressed")


    def callback(self, ros_data):
        global prev_time, fps, time_diff
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print( 'received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2RGBA)
        shape = image_np.shape


        imgOut = np.empty_like(image_np)

        h, w, d = shape


        imgInBuf = cl.Image(context, cl.mem_flags.READ_ONLY, cl.ImageFormat(cl.channel_order.RGBA, cl.channel_type.UNSIGNED_INT8), shape=(w,h))
        imgOutBuf = cl.Image(context, cl.mem_flags.WRITE_ONLY, cl.ImageFormat(cl.channel_order.RGBA, cl.channel_type.UNSIGNED_INT8), shape=(w,h))

        kernel.set_arg(0,imgInBuf)
        kernel.set_arg(1,imgOutBuf)
        kernel.set_arg(2,np.int32(h))
        kernel.set_arg(3,np.int32(w))

        cl.enqueue_copy(queue, imgInBuf, image_np, origin=(0, 0), region=(w,h))
        
        cl.enqueue_nd_range_kernel(queue, kernel, (w,h), None)

        cl.enqueue_copy(queue, imgOut, imgOutBuf, origin=(0, 0), region=(w,h))




        fps += 1
        time_now = time.time()
        time_diff += time_now -  prev_time
        if (time_diff) > 1:
            print("FPS:{}".format(fps))
            time_diff = fps = 0
        prev_time = time_now



        # linFilter = np.array([ [0,-1,2],
        #                     [0,-1,2],
        #                     [0,-1,2]])

        # output = cv2.filter2D(image_np, -1, linFilter)

        # # output = np.square(output)*10

        # output[:,:,0] = output[:,:,2]*2
        # output[:,:,1] = output[:,:,2]*3

        # output = output % 255



        # print(image_np[384,512,:])
        cv2.namedWindow("DepthCamera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("DepthCamera", 1024, 758)
        cv2.imshow('DepthCamera', imgOut)
        cv2.waitKey(1)



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