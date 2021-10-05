//
// Created by quantum on 10/4/21.
//

#include "IterativeClosestPoint.h"

const std::vector<uint32_t>& IterativeClosestPoint::CalculateAlignment(std::vector<float>& cloudOne, std::vector<float>& cloudTwo)
{

}

const std::vector<uint32_t>& IterativeClosestPoint::FindCorrespondences(std::vector<float>& cloudOne, std::vector<float>& cloudTwo)
{
   uint8_t cloudOneBuffer = _openCL->CreateLoadBufferFloat(cloudOne.data(), cloudOne.size());
   uint8_t cloudTwoBuffer = _openCL->CreateLoadBufferFloat(cloudTwo.data(), cloudTwo.size());

   _openCL->SetArgument("correspondenceKernel", 0, cloudOneBuffer);
   _openCL->SetArgument("correspondenceKernel", 1, cloudTwoBuffer);
   _openCL->SetArgumentInt("correspondenceKernel", 2, cloudOne.size());
   _openCL->SetArgumentInt("correspondenceKernel", 3, cloudTwo.size());

   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->correspondenceKernel, cl::NullRange, cl::NDRange(1000), cl::NullRange);

   _openCL->commandQueue.finish();

   return std::vector<uint32_t>();
}
