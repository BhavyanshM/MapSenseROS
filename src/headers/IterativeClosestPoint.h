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
      Eigen::Matrix4f CalculateAlignment(std::vector<float>& cloudOne, std::vector<float>& cloudTwo);
      const Eigen::Matrix4f& FindCorrespondences(std::vector<float>& cloudOne, std::vector<float>& cloudTwo);
      void SetOpenCLManager(OpenCLManager* ocl) {_openCL = ocl;}
      void TestICP(std::vector<float>& cloudOne, std::vector<float>& cloudTwo, std::vector<int>& matches);

   private:
      OpenCLManager* _openCL;

};

#endif //MAP_SENSE_ITERATIVECLOSESTPOINT_H
