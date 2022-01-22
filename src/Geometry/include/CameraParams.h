//
// Created by quantum on 12/16/21.
//

#ifndef MAP_SENSE_CAMERAPARAMS_H
#define MAP_SENSE_CAMERAPARAMS_H

#include "Eigen/Core"
#include "opencv4/opencv2/opencv.hpp"
#include "boost/format.hpp"

class CameraParams
{
   public:
      CameraParams() {};
      CameraParams(float fx, float fy, float cx, float cy);
      const Eigen::Vector2f& Project(const Eigen::Vector3f& point);

      const Eigen::Matrix4f& GetExtrinsicMatrix();
      const Eigen::Matrix3f& GetIntrinsicMatrix();

      void SetParams(float fx, float fy, float cx, float cy)
      {
         _fx = fx;
         _fy = fy;
         _cx = cx;
         _cy = cy;
      };

      float _fx, _fy, _cx, _cy;

   private:

      Eigen::Matrix3f _cameraMatrix;
      Eigen::Matrix4f _transform;

};

#endif //MAP_SENSE_CAMERAPARAMS_H
