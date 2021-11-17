//
// Created by quantum on 5/13/21.
//

#ifndef IMAGETOOLS_H
#define IMAGETOOLS_H


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <opencv4/opencv2/calib3d.hpp>

class ImageTools
{
   public:
      static cv::Mat cvUndistort(cv::Mat image, cv::Mat intrinsics, cv::Mat distortionCoefficients);
};

#endif //IMAGETOOLS_H
