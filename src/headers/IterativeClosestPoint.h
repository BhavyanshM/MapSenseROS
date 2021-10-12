//
// Created by quantum on 10/4/21.
//

#ifndef MAP_SENSE_ITERATIVECLOSESTPOINT_H
#define MAP_SENSE_ITERATIVECLOSESTPOINT_H

#include "OpenCLManager.h"
#include "Eigen/Dense"

class IterativeClosestPoint
{
   public:
      Eigen::Matrix4f CalculateAlignment(std::vector<float>& cloudOne, const Eigen::Matrix4f& transformOne, std::vector<float>& cloudTwo, const Eigen::Matrix4f& transformTwo);
      Eigen::Matrix4f CalculateTransform(std::vector<float>& cloudOne, std::vector<float>& cloudTwo, std::vector<int>& matchesVector);
      void SetOpenCLManager(OpenCLManager* ocl) {_openCL = ocl;}
      void TestICP(std::vector<float>& cloudOne, std::vector<float>& cloudTwo, std::vector<int>& matches);
      Eigen::Matrix3f CalculateCorrelationMatrix(uint32_t threads);

   private:
      OpenCLManager* _openCL;

};

#endif //MAP_SENSE_ITERATIVECLOSESTPOINT_H
