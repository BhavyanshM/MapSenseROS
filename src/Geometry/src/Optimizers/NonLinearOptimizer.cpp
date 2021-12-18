//
// Created by quantum on 12/2/21.
//

#include "NonLinearOptimizer.h"

NonLinearOptimizer::NonLinearOptimizer(uint32_t numCameras, uint32_t numPoints) : _numCameras(numCameras), _numPoints(numPoints)
{
   _numTotalParams = 6 * numCameras + 3 * numPoints;
   _params = Eigen::VectorXd::Zero(_numTotalParams);


}

void NonLinearOptimizer::Linearize()
{

}

void NonLinearOptimizer::Solve()
{
}

void NonLinearOptimizer::Update()
{
}
