import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread("/home/quantum/Workspace/FastStorage/catkin_ws/src/MapSenseROS/Extras/Images/Capture_Depth.png", cv2.IMREAD_GRAYSCALE)

img = 255 - img

cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Image", img.shape[1]*4, img.shape[0]*4)
cv2.imshow("Image", img)
cv2.waitKey(0)


print(img.ravel().tolist())
plt.hist(img.ravel(), bins=256, range=(0, 255), fc='k', ec='k')
plt.show()