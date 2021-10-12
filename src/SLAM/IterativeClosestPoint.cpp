//
// Created by quantum on 10/4/21.
//

#include "IterativeClosestPoint.h"

Eigen::Matrix4f IterativeClosestPoint::CalculateAlignment(std::vector<float>& cloudOne, const Eigen::Matrix4f& transformOne, std::vector<float>& cloudTwo, const Eigen::Matrix4f& transformTwo)
{
   auto start_point = std::chrono::steady_clock::now();

   std::vector<float> transformOneVec, transformTwoVec;
   for(int i = 0; i<9; i++)
   {
      transformOneVec.emplace_back(transformOne(i/3, i%3));
      transformTwoVec.emplace_back(transformTwo(i/3, i%3));
   }
   for(int i = 0; i<3 ;i++)
   {
      transformOneVec.emplace_back(transformOne(i, 3));
      transformTwoVec.emplace_back(transformTwo(i, 3));
   }

   int numPoints = cloudOne.size()/3;
   uint32_t threads = 1000;

   uint8_t cloudOneBuffer = _openCL->CreateLoadBufferFloat(cloudOne.data(), cloudOne.size());
   uint8_t transformOneBuffer = _openCL->CreateLoadBufferFloat(transformOneVec.data(), 12);
   uint8_t cloudTwoBuffer = _openCL->CreateLoadBufferFloat(cloudTwo.data(), cloudTwo.size());
   uint8_t transformTwoBuffer = _openCL->CreateLoadBufferFloat(transformTwoVec.data(), 12);
   uint8_t matchBuffer = _openCL->CreateBufferInt(numPoints);
   uint8_t correlBuffer = _openCL->CreateBufferFloat( 9 * threads);

   _openCL->SetArgument("correspondenceKernel", 0, cloudOneBuffer);
   _openCL->SetArgument("correspondenceKernel", 1, transformOneBuffer);
   _openCL->SetArgument("correspondenceKernel", 2, cloudTwoBuffer);
   _openCL->SetArgument("correspondenceKernel", 3, transformTwoBuffer);
   _openCL->SetArgument("correspondenceKernel", 4, matchBuffer);
   _openCL->SetArgumentInt("correspondenceKernel", 5, cloudOne.size());
   _openCL->SetArgumentInt("correspondenceKernel", 6, cloudTwo.size());
   _openCL->SetArgumentInt("correspondenceKernel", 7, numPoints);
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->correspondenceKernel, cl::NullRange, cl::NDRange(numPoints), cl::NullRange);

//   int matches[cloudOne.size()/3];
//   _openCL->ReadBufferInt(matchBuffer, matches, cloudOne.size()/3);

   // Setup kernel for calculating correlation matrix.
   _openCL->SetArgument("correlationKernel", 0, cloudOneBuffer);
   _openCL->SetArgument("correlationKernel", 1, transformOneBuffer);
   _openCL->SetArgument("correlationKernel", 2, cloudTwoBuffer);
   _openCL->SetArgument("correlationKernel", 3, transformTwoBuffer);
   _openCL->SetArgument("correlationKernel", 4, matchBuffer);
   _openCL->SetArgument("correlationKernel", 5, correlBuffer);
   _openCL->SetArgumentInt("correlationKernel", 6, cloudOne.size());
   _openCL->SetArgumentInt("correlationKernel", 7, cloudTwo.size());
   _openCL->SetArgumentInt("correlationKernel", 8, threads);

   /*TODO: Upload mean of both pointclouds and subtract before calculating correlation. */

   Eigen::Matrix3f correl = CalculateCorrelationMatrix(threads, correlBuffer);


//   std::vector<int> matchesVector(matches, matches + cloudOne.size()/3);

   // Calculate transform on the CPU.
//   Eigen::Matrix4f transform = CalculateTransform(cloudOne, cloudTwo, matchesVector);

   //Calculate transform on the GPU.


   auto end_point = std::chrono::steady_clock::now();
   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

   printf("Total Time Taken: %.3lf ms\n", duration);

//   TestICP(cloudOne, cloudTwo, matchesVector);

   return Eigen::Matrix4f::Identity();
}

Eigen::Matrix3f IterativeClosestPoint::CalculateCorrelationMatrix(uint32_t threads, uint8_t correlBuffer)
{
   float correlationMatrixBuffer[9 * threads];
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->correlationKernel, cl::NullRange, cl::NDRange((int)threads), cl::NullRange);
   _openCL->ReadBufferFloat(correlBuffer, correlationMatrixBuffer, 9 * threads);
   _openCL->commandQueue.finish();

   Eigen::Map<Eigen::Matrix<float, 9, Eigen::Dynamic>, Eigen::RowMajor> correlation(correlationMatrixBuffer, 9, threads);
   Eigen::MatrixXf correlMatVec = correlation.rowwise().sum();

   std::cout << "CorrelMatVec: " << correlMatVec << std::endl;

   return Eigen::Matrix3f::Identity();
}

Eigen::Matrix4f IterativeClosestPoint::CalculateTransform(std::vector<float>& cloudOne, std::vector<float>& cloudTwo, std::vector<int>& matchesVector)
{
   // Count number of corresponding point pairs found.
   int numOfCorrespondences = 0;
   for(int i = 0; i<cloudOne.size()/3; i++)
   {
      if(matchesVector[i] != -1 && matchesVector[i] != 0) numOfCorrespondences += 1;
   }
   printf("Total Correspondences: %d\n", numOfCorrespondences);

   // Create corresponding matrices for ICP correlation matrix calculation.
   Eigen::MatrixXf firstCloud(numOfCorrespondences, 3);
   Eigen::MatrixXf secondCloud(numOfCorrespondences, 3);
   int count = 0;
   for(int i = 0; i<cloudOne.size()/3; i++)
   {
      if(matchesVector[i] != -1 && matchesVector[i] != 0)
      {
         Eigen::Vector3f one(cloudOne[i*3], cloudOne[i*3 + 1], cloudOne[i*3 + 2]);
         Eigen::Vector3f two(cloudTwo[matchesVector[i]*3], cloudTwo[matchesVector[i]*3 + 1], cloudTwo[matchesVector[i]*3 + 2]);
         firstCloud(count, 0) = one(0);
         firstCloud(count, 1) = one(1);
         firstCloud(count, 2) = one(2);
         secondCloud(count, 0) = two(0);
         secondCloud(count, 1) = two(1);
         secondCloud(count, 2) = two(2);
         count++;
      }
   }

   // Calculate centroid vectors for both the pointclouds.
   Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>, Eigen::RowMajor> smallCloudOne(cloudOne.data(), 3,cloudOne.size()/3);
   Eigen::Map<Eigen::Matrix<float, 3, Eigen::Dynamic>, Eigen::RowMajor> smallCloudTwo(cloudTwo.data(), 3,cloudTwo.size()/3);
   Eigen::Vector3f smallMeanOne = smallCloudOne.rowwise().mean();
   Eigen::Vector3f smallMeanTwo = smallCloudTwo.rowwise().mean();
//   std::cout << "Small First Size: " << smallCloudOne.cols() << std::endl;

   // Calculate centroid vectors for both the pointclouds.
   Eigen::Vector3f firstMean = firstCloud.colwise().mean();
   Eigen::Vector3f secondMean = secondCloud.colwise().mean();
//   std::cout << "First Size: " << firstCloud.rows() << std::endl;

   // Remove centroid from both pointcloud matrices.
   firstCloud = firstCloud - firstMean.rowwise().replicate(firstCloud.rows()).transpose();
   secondCloud = secondCloud - secondMean.rowwise().replicate(secondCloud.rows()).transpose();

   // Calculate the correlation matrix.
   Eigen::Matrix3f correlation = firstCloud.transpose() * secondCloud;
   std::cout << "Correlation:" << correlation << std::endl;
   // Calculate SVD for the correlation matrix.
   Eigen::JacobiSVD<Eigen::MatrixXf> svd(correlation, Eigen::ComputeThinU | Eigen::ComputeThinV);

   // Recover rotation matrix from the SVD matrices.
   Eigen::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();
//   std::cout << "R:" << std::endl << rotation << std::endl;
//   std::cout << "t:" << std::endl << firstMean - secondMean << std::endl;

   // Construct a 4x4 transformation matrix from rotation and centroidal displacement.
   Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
   transform.block<3,3>(0,0) = rotation;
   transform.block<3,1>(0,3) = firstMean - secondMean;

   return transform;
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
//      printf("%d, %d\n", currentMatches[i], matches[i]);
      if(currentMatches[i] == matches[i])
      {
         count++;
      }
   }
   printf("Percent Correct Matches: %.2lf, Total Correct Matches: %d\n", 100.0 * (float)count/((float)currentMatches.size()), count);
}
