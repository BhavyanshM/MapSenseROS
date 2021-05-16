//
// Created by quantum on 5/13/21.
//

#ifndef IMAGETOOLS_H
#define IMAGETOOLS_H


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;

class ImageTools
{
   public:
      static Mat cvUndistort(Mat image, Mat intrinsics, Mat distortionCoefficients);
};

#endif //IMAGETOOLS_H
