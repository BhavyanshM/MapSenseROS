//
// Created by quantum on 10/4/21.
//

#include "IterativeClosestPoint.h"

const std::vector<uint32_t>& IterativeClosestPoint::CalculateAlignment(std::vector<float>& cloudOne, std::vector<float>& cloudTwo)
{
//   Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
//   int totalNumOfBoundaryPoints = 0;
//   for (int i = 0; i < this->matches.size(); i++)
//   {
//      totalNumOfBoundaryPoints += this->_latestRegionsZUp[this->matches[i].second]->getNumOfBoundaryVertices();
//   }
//   Eigen::MatrixXf A(totalNumOfBoundaryPoints, 6);
//   VectorXf b(totalNumOfBoundaryPoints);
//
//   int i = 0;
//   for (int m = 0; m < this->matches.size(); m++)
//   {
//      for (int n = 0; n < this->_latestRegionsZUp[this->matches[m].second]->getNumOfBoundaryVertices(); n++)
//      {
//         Vector3f latestPoint = _latestRegionsZUp[matches[m].second]->getVertices()[n];
//         //         printf("(%d,%d,%d):(%.2lf,%.2lf,%.2lf)\n", m,n, i, latestPoint.x(), latestPoint.y(), latestPoint.z());
//         Vector3f correspondingMapCentroid = regions[matches[m].first]->getCenter();
//         Vector3f correspondingMapNormal = regions[matches[m].first]->getNormal();
//         Vector3f cross = latestPoint.cross(correspondingMapNormal);
//         A(i, 0) = cross(0);
//         A(i, 1) = cross(1);
//         A(i, 2) = cross(2);
//         A(i, 3) = correspondingMapNormal(0);
//         A(i, 4) = correspondingMapNormal(1);
//         A(i, 5) = correspondingMapNormal(2);
//         b(i) = -(latestPoint - correspondingMapCentroid).dot(correspondingMapNormal);
//         i++;
//      }
//   }
//   VectorXf solution(6);
//   solution = A.bdcSvd(ComputeThinU | ComputeThinV).solve(b);
//   eulerAnglesToReference = Vector3d((double) solution(0), (double) solution(1), (double) solution(2));
//   translationToReference = Vector3d((double) solution(3), (double) solution(4), (double) solution(5));
//
//   /* Update relative and total transform from current sensor pose to map frame. Required for initial value for landmarks observed in current pose. */
//   _sensorPoseRelative = RigidBodyTransform(eulerAnglesToReference, translationToReference);
//   _sensorToMapTransform.multiplyRight(_sensorPoseRelative);
}

const std::vector<uint32_t>& IterativeClosestPoint::FindCorrespondences(std::vector<float>& cloudOne, std::vector<float>& cloudTwo)
{
   auto start_point = std::chrono::steady_clock::now();


   uint8_t cloudOneBuffer = _openCL->CreateLoadBufferFloat(cloudOne.data(), cloudOne.size());
   uint8_t cloudTwoBuffer = _openCL->CreateLoadBufferFloat(cloudTwo.data(), cloudTwo.size());
   uint8_t matchBuffer = _openCL->CreateBufferInt(cloudOne.size() / 3);

   printf("BufferMatchSize(%d)\n", cloudOne.size() / 3);

   int numPoints = cloudOne.size()/3;

   _openCL->SetArgument("correspondenceKernel", 0, cloudOneBuffer);
   _openCL->SetArgument("correspondenceKernel", 1, cloudTwoBuffer);
   _openCL->SetArgument("correspondenceKernel", 2, matchBuffer);
   _openCL->SetArgumentInt("correspondenceKernel", 3, cloudOne.size());
   _openCL->SetArgumentInt("correspondenceKernel", 4, cloudTwo.size());
   _openCL->SetArgumentInt("correspondenceKernel", 5, numPoints);
   _openCL->SetArgumentInt("correspondenceKernel", 6, 1);

   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->correspondenceKernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange);

   int matches[cloudOne.size()/3];
   _openCL->ReadBuffer(matchBuffer, matches, cloudOne.size()/3);

   _openCL->commandQueue.finish();



   int numOfCorrespondences = 0;
   for(int i = 0; i<cloudOne.size()/3; i++)
   {
      if(matches[i] != -1 && matches[i] != 0) numOfCorrespondences += 1;

//      if((one-two).norm() > 1.0) zeroes += 1;
//      if((one-two).norm() > 2.0)printf("Match: (%d,%d) Dist:%.3lf\n", i, matches[i], (one-two).norm());
   }
   printf("Total Correspondences: %d\n", numOfCorrespondences);

   Eigen::MatrixXf firstCloud(numOfCorrespondences, 3);
   Eigen::MatrixXf secondCloud(numOfCorrespondences, 3);
   int count = 0;
   for(int i = 0; i<cloudOne.size()/3; i++)
   {
      if(matches[i] != -1 && matches[i] != 0)
      {
         Eigen::Vector3f one(cloudOne[i*3], cloudOne[i*3 + 1], cloudOne[i*3 + 2]);
         Eigen::Vector3f two(cloudTwo[matches[i]*3], cloudTwo[matches[i]*3 + 1], cloudTwo[matches[i]*3 + 2]);
         firstCloud(count, 0) = one(0);
         firstCloud(count, 1) = one(1);
         firstCloud(count, 2) = one(2);
         secondCloud(count, 0) = two(0);
         secondCloud(count, 1) = two(1);
         secondCloud(count, 2) = two(2);
         count++;
      }

   }

   Eigen::Vector3f firstMean = firstCloud.colwise().mean();
   Eigen::Vector3f secondMean = secondCloud.colwise().mean();
   std::cout << "First Mean: " << firstMean << std::endl;
   std::cout << "Second Mean: " << secondMean << std::endl;

   firstCloud = firstCloud - firstMean.rowwise().replicate(firstCloud.rows()).transpose();
   secondCloud = secondCloud - secondMean.rowwise().replicate(secondCloud.rows()).transpose();



   Eigen::Matrix3f correlation = firstCloud.transpose() * secondCloud;

   std::cout << "Correlation Matrix:" << correlation << std::endl;

   auto end_point = std::chrono::steady_clock::now();
   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

   printf("Total Time Taken: %.3lf ms\n", duration);

   return std::vector<uint32_t>();
}
