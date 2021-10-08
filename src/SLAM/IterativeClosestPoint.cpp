//
// Created by quantum on 10/4/21.
//

#include "IterativeClosestPoint.h"

Eigen::Matrix4f IterativeClosestPoint::CalculateAlignment(std::vector<float>& cloudOne, std::vector<float>& cloudTwo)
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

   std::vector<int> matchesVector(matches, matches + cloudOne.size()/3);

   int numOfCorrespondences = 0;
   for(int i = 0; i<cloudOne.size()/3; i++)
   {
      if(matches[i] != -1 && matches[i] != 0) numOfCorrespondences += 1;

      //      if((one-two).norm() > 1.0) zeroes += 1;
//      if(matches[i] != -1 && matches[i] != 0) printf("Match: (%d,%d)\n", i, matches[i]);
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

   Eigen::JacobiSVD<Eigen::MatrixXf> svd(correlation, Eigen::ComputeThinU | Eigen::ComputeThinV);

   std::cout << "S:" << std::endl << svd.singularValues() << std::endl;
   std::cout << "U:" << std::endl << svd.matrixU() << std::endl;
   std::cout << "V:" << std::endl << svd.matrixV() << std::endl;

   Eigen::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();
   std::cout << "R:" << std::endl << rotation << std::endl;
   std::cout << "t:" << std::endl << firstMean - secondMean << std::endl;

   Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
   transform.block<3,3>(0,0) = rotation.transpose();
   transform.block<3,1>(0,3) = firstMean - secondMean;

   auto end_point = std::chrono::steady_clock::now();
   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

   printf("Total Time Taken: %.3lf ms\n", duration);

   TestICP(cloudOne, cloudTwo, matchesVector);

   return transform;
}

const Eigen::Matrix4f& IterativeClosestPoint::FindCorrespondences(std::vector<float>& cloudOne, std::vector<float>& cloudTwo)
{
   return Eigen::Matrix4f::Identity();
}

void IterativeClosestPoint::TestICP(std::vector<float>& cloudOne, std::vector<float>& cloudTwo, std::vector<int>& matches)
{
   std::vector<int> currentMatches;
   for(int i = 0; i<cloudOne.size()/3; i++)
   {
      float minLength = 1000000000;
      int minindex = -1;
      for(int j = 0; j<cloudTwo.size()/3; j++)
      {
         Eigen::Vector3f one(cloudOne[i*3], cloudOne[i*3+1], cloudOne[i*3+2]);
         Eigen::Vector3f two(cloudTwo[j*3], cloudTwo[j*3+1], cloudTwo[j*3+2]);
         if((one - two).norm() < minLength)
         {
            minLength = (one - two).norm();
            minindex = j;
         }
      }
      currentMatches.emplace_back(minindex);
   }
   int count = 0;
   for(int i = 0; i<currentMatches.size(); i++)
   {
      printf("%d, %d\n", currentMatches[i], matches[i]);
      if(currentMatches[i] == matches[i])
      {
         count++;
      }
   }
   printf("Percent Correct Matches: %.2lf, Total Correct Matches: %d\n", 100.0 * (float)count/((float)currentMatches.size()), count);
}
