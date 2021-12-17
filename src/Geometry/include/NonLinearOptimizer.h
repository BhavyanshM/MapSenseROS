//
// Created by quantum on 12/2/21.
//

#ifndef MAP_SENSE_NONLINEAROPTIMIZER_H
#define MAP_SENSE_NONLINEAROPTIMIZER_H

#include "Eigen/Core"
#include "Eigen/Sparse"

class NonLinearOptimizer
{
   public:
      NonLinearOptimizer(uint32_t numCameras, uint32_t numPoints);
      void Initialize();
      void Linearize();
      void Solve();
      void Update();


   private:
      uint32_t _numCameras = 0;
      uint32_t _numPoints = 0;
      uint32_t _numTotalParams = 0;

      float _totalError = MAXFLOAT;
      uint8_t iterations = 10;

      Eigen::MatrixXd _J;
      Eigen::MatrixXd _JtJ;
      Eigen::VectorXd _params;
};

#endif //MAP_SENSE_NONLINEAROPTIMIZER_H
