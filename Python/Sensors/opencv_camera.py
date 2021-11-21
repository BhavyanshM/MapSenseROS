import cv2


def set_cam_props(cam):
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cam.set(cv2.CAP_PROP_FPS, 30)
    cam.set(cv2.CAP_PROP_MODE, cv2.CAP_OPENCV_MJPEG)
    # cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    # cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    # cam.set(cv2.CAP_PROP_EXPOSURE, -4)


if __name__ == "__main__":
    camLeft = cv2.VideoCapture(0, cv2.CAP_V4L2)
    set_cam_props(camLeft)
    print("Left Camera Available: ", camLeft.isOpened())

    camRight = cv2.VideoCapture(2, cv2.CAP_V4L2)
    set_cam_props(camRight)
    print("Right Camera Available: ", camRight.isOpened())

    while True:
        _, imgLeft = camLeft.read()
        _, imgRight = camRight.read()

        final = cv2.hconcat([imgLeft, imgRight])

        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.imshow("Image", final)
        cv2.resizeWindow("Image", final.shape[1]*2, final.shape[0]*2)
        code = cv2.waitKeyEx(1)

        if code == 113:
            camLeft.release()
            camRight.release()
            cv2.destroyAllWindows()
            exit()


