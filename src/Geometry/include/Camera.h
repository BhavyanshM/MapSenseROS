//
// Created by quantum on 12/16/21.
//

#ifndef MAP_SENSE_CAMERA_H
#define MAP_SENSE_CAMERA_H

#include "Eigen/Core"

class Camera
{
   public:
      Camera(float fx, float fy, float cx, float cy);
      const Eigen::Vector2f& Project(const Eigen::Vector3f& point);

   private:
      float _fx, _fy, _cx, _cy;
      Eigen::Matrix4f _cameraMatrix;
      Eigen::Matrix4f _transform;
};

#endif //MAP_SENSE_CAMERA_H
