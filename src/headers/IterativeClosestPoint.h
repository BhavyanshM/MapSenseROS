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
      const std::vector<uint32_t>& CalculateAlignment(std::vector<float>& cloudOne, std::vector<float>& cloudTwo);
      const std::vector<uint32_t>& FindCorrespondences(std::vector<float>& cloudOne, std::vector<float>& cloudTwo);
      void SetOpenCLManager(OpenCLManager* ocl) {_openCL = ocl;}

   private:
      OpenCLManager* _openCL;

};

#endif //MAP_SENSE_ITERATIVECLOSESTPOINT_H
