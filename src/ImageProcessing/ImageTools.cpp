//
// Created by quantum on 5/13/21.
//

#include "ImageTools.h"

Mat ImageTools::cvUndistort(Mat image, Mat intrinsics, Mat distortionCoefficients)
{
   cv::Mat imageUndistorted;
   undistort(image, imageUndistorted, intrinsics, distortionCoefficients);
   return imageUndistorted;
}