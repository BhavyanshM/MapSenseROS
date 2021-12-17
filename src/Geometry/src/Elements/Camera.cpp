//
// Created by quantum on 12/16/21.
//

#include "Camera.h"

Camera::Camera(float fx, float fy, float cx, float cy)  : _fx(fx), _fy(fy), _cx(cx), _cy(cy)
{

}

const Eigen::Vector2f& Camera::Project(const Eigen::Vector3f& point)
{

}

