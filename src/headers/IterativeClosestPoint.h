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
      Eigen::Matrix4f CalculateAlignment(std::vector<float>& cloudOne, const Eigen::Matrix4f& transformOne, std::vector<float>& cloudTwo, const Eigen::Matrix4f& transformTwo, int *partIds = nullptr, int partCount = 32);
      Eigen::Matrix4f CalculateTransformSequential(std::vector<float>& cloudOne, std::vector<float>& cloudTwo, std::vector<int>& matchesVector);
      void SetOpenCLManager(OpenCLManager* ocl) {_openCL = ocl;}
      void TestICP(std::vector<float>& cloudOne, std::vector<float>& cloudTwo, std::vector<int>& matches);
      Eigen::Matrix4f CalculateTransformParallel(uint32_t threads, uint8_t correlBuffer, uint8_t meanBuffer);
      Eigen::Matrix4f ExtractTransform(Eigen::Matrix3f correlation, Eigen::Vector3f meanOne, Eigen::Vector3f meanTwo);

   private:
      OpenCLManager* _openCL;
      uint8_t _iteration = 0;
      const uint8_t MAX_STEPS = 8;

};

#endif //MAP_SENSE_ITERATIVECLOSESTPOINT_H
