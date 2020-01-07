import sys, time# numpy and scipy
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
import pyopencl as cl

# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

context = cl.Context([cl.get_platforms()[0].get_devices()[0]])
queue = cl.CommandQueue(context)
program = cl.Program(context, open('kernel.cl').read()).build()
kernel = cl.Kernel(program, 'depthKernel')




class image_feature:

    def __init__(self):
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/depth_image/compressed",
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

        shape = image_np.T.shape
        imgOut = np.empty_like(image_np)


        imgInBuf = cl.Image(context, cl.mem_flags.READ_ONLY, cl.ImageFormat(cl.channel_order.RGB, cl.channel_type.UNORM_INT8), shape=shape)
        imgOutBuf = cl.Image(context, cl.mem_flags.WRITE_ONLY, cl.ImageFormat(cl.channel_order.LUMINANCE, cl.channel_type.UNORM_INT8), shape=shape)


        kernel.set_arg(0, imgInBuf)
        kernel.set_arg(1, imgOutBuf)

        cl.enqueue_copy(queue, imgInBuf, image_np, origin=(0, 0, 0), region=shape, is_blocking=False)
        cl.enqueue_nd_range_kernel(queue, kernel, shape, None)
        cl.enqueue_copy(queue, imgOut, imgOutBuf, origin=(0, 0, 0), region=shape, is_blocking=True)







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
        cv2.waitKey(0)



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