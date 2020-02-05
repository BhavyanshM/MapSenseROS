import sys, time# numpy and scipy
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from map_sense.msg import PlanarRegion
import pyopencl as cl
import time
# from cv_bridge import CvBridge, CvBridgeError
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

VERBOSE=False

context = cl.Context([cl.get_platforms()[0].get_devices()[0]])
queue = cl.CommandQueue(context,properties=cl.command_queue_properties.PROFILING_ENABLE)
program = cl.Program(context, open('fitting_kernel.cl').read()).build()
depthKernel = cl.Kernel(program, 'depthKernel')
segmentKernel = cl.Kernel(program, 'segmentKernel')

prev_time = time.time()
fps = 0
time_diff = 0
disp = 0
h, w, d = 768,1024,4
subH, subW, subD = 48,64,4


imgInBuf = cl.Image(context, cl.mem_flags.READ_WRITE, cl.ImageFormat(cl.channel_order.RGBA, cl.channel_type.UNSIGNED_INT8), shape=(w,h))

imgMedBuf = cl.Image(context, cl.mem_flags.READ_WRITE, cl.ImageFormat(cl.channel_order.RGBA, cl.channel_type.UNSIGNED_INT8), shape=(w,h))

imgOutBuf1 = cl.Image(context, cl.mem_flags.READ_WRITE, cl.ImageFormat(cl.channel_order.RGBA, cl.channel_type.FLOAT), shape=(subW, subH))
imgOutBuf2 = cl.Image(context, cl.mem_flags.READ_WRITE, cl.ImageFormat(cl.channel_order.RGBA, cl.channel_type.FLOAT), shape=(subW, subH))


depthKernel.set_arg(0,imgInBuf)
depthKernel.set_arg(1,imgMedBuf)
depthKernel.set_arg(2,np.int32(h))
depthKernel.set_arg(3,np.int32(w))

segmentKernel.set_arg(0,imgInBuf)
segmentKernel.set_arg(1,imgOutBuf1)
segmentKernel.set_arg(2,imgOutBuf2)
segmentKernel.set_arg(3,np.int32(subH))
segmentKernel.set_arg(4,np.int32(subW))

class image_feature:

    def __init__(self):
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/depth_image/compressed",
            CompressedImage, self.callback,  queue_size = 1)
        
        self.publisher = rospy.Publisher("/map/regions", CompressedImage, queue_size=10)
        
        if VERBOSE :
            print( "subscribed to /camera/image/compressed")


    def fit(self, image_np):
        global imgInBuf, imgOutBuf1, imgOutBuf2, depthKernel, segmentKernel
        imgMed = np.empty_like(image_np)
        imgOut1 = np.empty((48,64,4), dtype='float32')
        imgOut2 = np.empty((48,64,4), dtype='float32')

        cl.enqueue_copy(queue, imgInBuf, image_np, origin=(0, 0), region=(w,h), is_blocking=False)        
        cl.enqueue_nd_range_kernel(queue, depthKernel, (w,h), None)
        # cl.enqueue_copy(queue, imgMed, imgMedBuf, origin=(0, 0), region=(w,h), is_blocking=False)
        event = cl.enqueue_nd_range_kernel(queue, segmentKernel, (subW,subH), None)
        event.wait()
        print ((event.profile.end-event.profile.start)*(1e-6))
        cl.enqueue_copy(queue, imgOut1, imgOutBuf1, origin=(0, 0), region=(subW,subH), is_blocking=False)
        cl.enqueue_copy(queue, imgOut2, imgOutBuf2, origin=(0, 0), region=(subW,subH), is_blocking=False)

        cl.enqueue_barrier(queue)

        return imgOut1, imgOut2, imgMed

    def display(self, disp, image_np, imgMed, imgOut):
        cv2.namedWindow("DepthCamera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("DepthCamera", 1024, 758)
        if disp == 0:
            cv2.imshow('DepthCamera', image_np[:,:,2])
        if disp == 1:
            cv2.imshow('DepthCamera', imgMed)
        if disp == 2:
            cv2.imshow('DepthCamera', imgOut)


    def publish(self, imgOut):
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "png"
        msg.data = list(np.array(cv2.imencode('.png', imgOut)[1]))
        # print(len(msg.data))
        self.publisher.publish(msg)

    def capture(self, disp):
        code = cv2.waitKeyEx(1)
        # print(imgOut[0,0,:])
        if code == 1113937 or code == 65361:
            disp = (disp - 1) % 3
        if code == 1113939  or code == 65363 :
            disp = (disp + 1) % 3
        if code == 115  or code == 6536 :
            np.save("depth",image_np)

        return disp

    def callback(self, ros_data):
        global prev_time, fps, time_diff, disp 
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print( 'received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2RGBA)
        
        imgOut1, imgOut2, imgMed = self.fit(image_np)

        # print(image_np[0,0,2], imgOut[0,0,2], imgMed.dtype, imgOut.dtype)

        self.publish(imgOut1)
        disp = self.capture(disp)


        fps += 1
        time_now = time.time()
        time_diff += time_now -  prev_time
        if (time_diff) > 1:
            # print("FPS:{}".format(fps))
            time_diff = fps = 0
        prev_time = time_now

        # print(imgOut1[24,32,:])
        # print(imgOut2[24,32,:])


        self.display(disp, image_np, imgMed, imgOut1)




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