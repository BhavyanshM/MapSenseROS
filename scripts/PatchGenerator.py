import sys, time# numpy and scipy
import numpy as np
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from map_sense.msg import PlanarRegions
import pyopencl as cl
import time
from std_msgs.msg import Float32



# from cv_bridge import CvBridge, CvBridgeError
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

VERBOSE=False

context = cl.Context([cl.get_platforms()[0].get_devices()[0]])
queue = cl.CommandQueue(context,properties=cl.command_queue_properties.PROFILING_ENABLE)
program = cl.Program(context, open('../kernels/fitting_kernel.cpp').read()).build()
depthKernel = cl.Kernel(program, 'depthKernel')
segmentKernel = cl.Kernel(program, 'segmentKernel')

prev_time = time.time()
fps = 0
time_diff = 0
disp = 0
h, w, d = 768,1024,4
subH, subW, subD = 12,16,4


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
# segmentKernel.set_arg(3,np.int32(subH))
# segmentKernel.set_arg(4,np.int32(subW))

count = 0




def fit( image_np):
    global imgInBuf, imgOutBuf1, imgOutBuf2, depthKernel, segmentKernel
    imgMed = np.empty_like(image_np)
    imgOut1 = np.empty((subW,subH,4), dtype='float32')
    imgOut2 = np.empty((subW,subH,4), dtype='float32')

    cl.enqueue_copy(queue, imgInBuf, image_np, origin=(0, 0), region=(w,h), is_blocking=False)        
    cl.enqueue_nd_range_kernel(queue, depthKernel, (w,h), None)
    # cl.enqueue_copy(queue, imgMed, imgMedBuf, origin=(0, 0), region=(w,h), is_blocking=False)
    event = cl.enqueue_nd_range_kernel(queue, segmentKernel, (subW,subH), None)
    event.wait()
    # print ((event.profile.end-event.profile.start)*(1e-6))
    cl.enqueue_copy(queue, imgOut1, imgOutBuf1, origin=(0, 0), region=(subW,subH), is_blocking=False)
    cl.enqueue_copy(queue, imgOut2, imgOutBuf2, origin=(0, 0), region=(subW,subH), is_blocking=False)

    cl.enqueue_barrier(queue)

    return imgOut1, imgOut2, imgMed

def display( disp, image_np, imgMed, imgOut):
    cv2.namedWindow("DepthCamera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("DepthCamera", 1024, 758)
    if disp == 0:
        cv2.imshow('DepthCamera', image_np[:,:,2])
    if disp == 1:
        cv2.imshow('DepthCamera', imgMed)
    if disp == 2:
        cv2.imshow('DepthCamera', imgOut)


def publish( imgOut):

	print(imgOut.shape)

	imgOut = imgOut.flatten()

	np.savetxt("patches.csv", imgOut, delimiter=",")

	# data = imgOut.tolist()
	# data = list(np.array(cv2.imencode('.png', imgOut)[1]))





def main(args):
	path = "/home/quantum/Workspace/Storage/IHMC_PhD/Research/SurfaceExtraction/val/indoors/scene_00019/scan_00183/"
    
	img = cv2.imread(path+"00019_00183_indoors_350_050.png")

	image_np = np.load(path + "00019_00183_indoors_350_050_depth.npy")
	image_np = (image_np/np.max(image_np))*65536
	
	image_low = image_np % 256
	image_high = (image_np - image_low)/256

	print("High Mean:",np.mean(image_high))
	print("High StDev:",np.std(image_high))

	print("Low Mean:",np.mean(image_low))
	print("Low StDev:",np.std(image_low))

	imgStacked = np.concatenate((	image_low.astype(np.uint8), 
									image_high.astype(np.uint8),
									image_low.astype(np.uint8), 
									image_high.astype(np.uint8)	), axis=2)



	print(imgStacked.shape)

	# image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2RGBA)


	imgOut1, imgOut2, imgMed = fit(imgStacked)
	outputStacked = np.concatenate((imgOut1, imgOut2),axis=2)
	
	publish(outputStacked)
    
	print("outputStacked:", outputStacked[0,0,:])
    

	cv2.imshow("Image", img)
	cv2.imshow("Depth", image_high.astype(np.uint8)*10)
	k = cv2.waitKeyEx(0)

if __name__ == '__main__':
    main(sys.argv)