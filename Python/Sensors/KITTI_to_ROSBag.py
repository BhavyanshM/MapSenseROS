import rospy
from sensor_msgs.msg import PointCloud2, Image, CompressedImage
import os
import cv2
import numpy as np
import ros_numpy

class DatasetNode:
    def __init__(self):
        self.leftImgPub = rospy.Publisher('/kitti/left/image_rect/compressed', CompressedImage, queue_size=2)
        self.rightImagePub = rospy.Publisher('/kitti/right/image_rect/compressed', CompressedImage, queue_size=2)
        self.lidarPub = rospy.Publisher('/kitti/lidar/points', PointCloud2, queue_size=2)

        rospy.init_node('dataset_publisher', anonymous=True)
        self.rate = rospy.Rate(20) # 10hz
        self.counter = 0
        self.leftImagesPath = '/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_0/'
        self.rightImagesPath = '/home/quantum/Workspace/Storage/Other/Temp/dataset/sequences/00/image_1/'
        self.lidarPath = '/home/quantum/Workspace/Storage/Other/Temp/dataset/velodyne_dataset/sequences/00/velodyne/'
        self.leftImgFiles = sorted(os.listdir(self.leftImagesPath))
        self.rightImgFiles = sorted(os.listdir(self.rightImagesPath))
        self.lidarFiles = sorted(os.listdir(self.lidarPath))

    def CreateCompressedImage(self, input_image):
        img = CompressedImage()
        img.header.stamp = rospy.Time.now()
        img.format = "jpeg"
        img.data = np.array(cv2.imencode('.jpg', np.array(input_image))[1]).tostring()
        return img, input_image

    def GetPointCloudFromFile(self, file):
        scan = np.fromfile(file, dtype=np.float32)
        scan = np.array(scan, dtype=np.float64)
        scan = scan.reshape((-1, 4))
        xyz = scan[:, :3]
        # xyz = xyz[::4]
        msg = PointCloud2()
        msg.width = 2000
        msg.height = 64
        data = np.zeros(xyz.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        data['x'] = xyz[:, 0]
        data['y'] = xyz[:, 1]
        data['z'] = xyz[:, 2]
        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header.frame_id = 'velodyne'
        return msg, xyz

    def Run(self):
        while not rospy.is_shutdown():

            leftImg, leftImgRaw = self.CreateCompressedImage(cv2.imread(self.leftImagesPath + self.leftImgFiles[self.counter]))
            rightImg, rightImgRaw = self.CreateCompressedImage(cv2.imread(self.rightImagesPath + self.rightImgFiles[self.counter]))
            lidarMsg, lidarPoints = self.GetPointCloudFromFile(self.lidarPath + self.lidarFiles[self.counter])

            self.leftImgPub.publish(leftImg)
            self.rightImagePub.publish(rightImg)
            self.lidarPub.publish(lidarMsg)

            self.rate.sleep()
            self.counter += 1

            cv2.imshow('Image', leftImgRaw)
            code = cv2.waitKeyEx(1)
            if code == 113:
                exit()

if __name__ == '__main__':
    try:
        node = DatasetNode()
        node.Run()
    except rospy.ROSInterruptException:
        pass