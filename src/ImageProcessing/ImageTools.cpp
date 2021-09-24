//
// Created by quantum on 5/13/21.
//

#include "ImageTools.h"

cv::Mat ImageTools::cvUndistort(cv::Mat image, cv::Mat intrinsics, cv::Mat distortionCoefficients)
{

   /*
      image_width: 1280
      image_height: 1024
      camera_name: 12502226
      camera_matrix: (3, 3): data: [305.6744323977819, 0, 640.9175104495503, 0, 307.6945377439216, 512.9095529465322, 0, 0, 1]
      distortion_model: rational_polynomial
      distortion_coefficients: (1, 8): data: [-0.1304880574839372, 0.0343337720836711, 0, 0, 0.002347490605947351, 0.163868408051474, -0.02493286434834704, 0.01394671162254435]
      rectification_matrix: (3, 3): data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
      projection_matrix: (3, 4): data: [59.01958465576172, 0, 640.8088034578032, 0, 0, 72.80045318603516, 513.8869098299074, 0, 0, 0, 1, 0]
   */

   double cameraMatrixArray[9] = {305.6744323977819, 0, 640.9175104495503, 0, 307.6945377439216, 512.9095529465322, 0, 0, 1};
   cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, cameraMatrixArray);

   double distCoeffArray[9] = {-0.1304880574839372, 0.0343337720836711, 0, 0, 0.002347490605947351, 0.163868408051474, -0.02493286434834704, 0.01394671162254435};
   cv::Mat distCoeffMatrix = cv::Mat(8, 1, CV_64F, distCoeffArray);

   cv::Mat imageUndistorted;
   undistort(image, imageUndistorted, cameraMatrix, distCoeffMatrix);

   return imageUndistorted;
}