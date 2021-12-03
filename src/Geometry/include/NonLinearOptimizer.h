//
// Created by quantum on 12/2/21.
//

#ifndef MAP_SENSE_NONLINEAROPTIMIZER_H
#define MAP_SENSE_NONLINEAROPTIMIZER_H

#include "Eigen/Core"
#include "Eigen/Sparse"

class NonLinearOptimizer
{
      void Initialize();
      void Linearize();
      void Solve();
      void Update();

      Eigen::MatrixXd J;
      Eigen::MatrixXd JtJ;
      Eigen::VectorXd params;
};

#endif //MAP_SENSE_NONLINEAROPTIMIZER_H
