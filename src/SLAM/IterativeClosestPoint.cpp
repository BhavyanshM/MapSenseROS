#include <iostream>
#include "IterativeClosestPoint.h"
#include "chrono"

Eigen::Matrix4f IterativeClosestPoint::CalculateAlignment(std::vector<float>& cloudOne, const Eigen::Matrix4f& transformOne, std::vector<float>& cloudTwo, const Eigen::Matrix4f& transformTwo, int* partIds, int partCount, int numVertBlocks)
{
//   if(_iteration > MAX_STEPS) return Eigen::Matrix4f::Identity(); else _iteration++;

   // Create std::vectors for initial pointcloud transforms to be uploaded to OpenCL kernel
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

   // Define some quantities as variables
   int numPointsOne = cloudOne.size()/3;
   int numPointsTwo = cloudTwo.size()/3;
   int numCylinderParts = partCount;

   int numBlocksTotal = numCylinderParts * numVertBlocks;
   uint32_t threads = 1000;

   // Create OpenCL Buffers for Iterative Closest Point kernels. (Allocate ~320 KB, less than 1 MB)
   uint8_t cloudOneBuffer = _openCL->CreateLoadBufferFloat(cloudOne.data(), cloudOne.size()); // ~80,000 * 3 * 4 bytes
   uint8_t transformOneBuffer = _openCL->CreateLoadBufferFloat(transformOneVec.data(), 12); // ~12 * 4 bytes
   uint8_t cloudTwoBuffer = _openCL->CreateLoadBufferFloat(cloudTwo.data(), cloudTwo.size()); // ~80,000 * 3 * 4 bytes
   uint8_t transformTwoBuffer = _openCL->CreateLoadBufferFloat(transformTwoVec.data(), 12); // ~12 * 4 bytes
   uint8_t matchBuffer = _openCL->CreateBufferInt(numPointsOne); // ~80,000 * 4 bytes
   uint8_t correlBuffer = _openCL->CreateBufferFloat( 9 * threads); // ~10,000 * 4 bytes
   uint8_t meanBuffer = _openCL->CreateBufferFloat(6 * threads); // ~3,000 * 4 bytes
   uint8_t normalBuffer = _openCL->CreateBufferFloat(3 * numBlocksTotal); // ~60 * 4 bytes
   uint8_t cylinderIndexBufferTwo = _openCL->CreateBufferInt(numPointsTwo); // ~80,000 * 4 bytes
   uint8_t cylinderBlockIdBufferTwo = _openCL->CreateBufferInt(numPointsTwo); // ~80,000 * 4 bytes
   uint8_t blockPointCount = _openCL->CreateBufferInt(numBlocksTotal); // ~60 * 4 bytes

   // Set arguments for the Cylinder Part generation kernel for only the latest pointcloud. Preserve cylinder buffer for later.
   _openCL->SetArgument("cylinderKernel", 0, cloudTwoBuffer);
   _openCL->SetArgument("cylinderKernel", 1, cylinderIndexBufferTwo);
   _openCL->SetArgumentInt("cylinderKernel", 2, numPointsTwo);
   _openCL->SetArgumentInt("cylinderKernel", 3, numCylinderParts);
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->cylinderKernel, cl::NullRange, cl::NDRange(numCylinderParts), cl::NullRange);

   // Set arguments for the plane calculation kernel. Pass cylinder parts for only the latest cloud.
   _openCL->SetArgument("planesKernel", 0, cloudTwoBuffer);
   _openCL->SetArgument("planesKernel", 1, cylinderIndexBufferTwo);
   _openCL->SetArgument("planesKernel", 2, cylinderBlockIdBufferTwo);
   _openCL->SetArgument("planesKernel", 3, normalBuffer);
   _openCL->SetArgument("planesKernel", 4, blockPointCount);
   _openCL->SetArgumentInt("planesKernel", 5, numPointsTwo);
   _openCL->SetArgumentInt("planesKernel", 6, numCylinderParts);
   _openCL->SetArgumentInt("planesKernel", 7, numVertBlocks);
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->planesKernel, cl::NullRange, cl::NDRange(numCylinderParts), cl::NullRange);

   // Set arguments for Correspondence Calculation kernel. Pass the surface normal buffers for both clouds.
   _openCL->SetArgument("correspondenceKernel", 0, cloudOneBuffer);
   _openCL->SetArgument("correspondenceKernel", 1, transformOneBuffer);
   _openCL->SetArgument("correspondenceKernel", 2, cloudTwoBuffer);
   _openCL->SetArgument("correspondenceKernel", 3, transformTwoBuffer);
   _openCL->SetArgument("correspondenceKernel", 4, matchBuffer);
   _openCL->SetArgumentInt("correspondenceKernel", 5, cloudOne.size());
   _openCL->SetArgumentInt("correspondenceKernel", 6, cloudTwo.size());
   _openCL->SetArgumentInt("correspondenceKernel", 7, numPointsOne);
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->correspondenceKernel, cl::NullRange, cl::NDRange(numPointsOne), cl::NullRange);

   // Set arguments for Centroid calculation kernel. Calculates centroids for both clouds at once. Could potentially store & reuse centroid for first cloud.
   _openCL->SetArgument("centroidKernel", 0, cloudOneBuffer);
   _openCL->SetArgument("centroidKernel", 1, cloudTwoBuffer);
   _openCL->SetArgument("centroidKernel", 2, meanBuffer);
   _openCL->SetArgument("centroidKernel", 3, matchBuffer);
   _openCL->SetArgumentInt("centroidKernel", 4, cloudOne.size());
   _openCL->SetArgumentInt("centroidKernel", 5, cloudTwo.size());
   _openCL->SetArgumentInt("centroidKernel", 6, threads);

   // Set arguments for the Correlation calculation kernel. Pass the point correspondences and surface normals for both clouds.
   _openCL->SetArgument("correlationKernel", 0, cloudOneBuffer);
   _openCL->SetArgument("correlationKernel", 1, cloudTwoBuffer);
   _openCL->SetArgument("correlationKernel", 2, meanBuffer);
   _openCL->SetArgument("correlationKernel", 3, matchBuffer);
   _openCL->SetArgument("correlationKernel", 4, correlBuffer);
   _openCL->SetArgumentInt("correlationKernel", 5, cloudOne.size());
   _openCL->SetArgumentInt("correlationKernel", 6, cloudTwo.size());
   _openCL->SetArgumentInt("correlationKernel", 7, threads);



   // Set partIds for visualization of cylinder parts in the UI.
   if(partIds != nullptr)
   {
      _openCL->ReadBufferInt(cylinderBlockIdBufferTwo, partIds, numPointsTwo);
//      std::vector<int> partIdsVec(partIds, partIds + numPointsOne);
//      for(int i = 0; i<numPointsOne; i++)
//      {
//         std::cout << "Index:" << i << " Id:" << partIdsVec[i] << std::endl;
//      }
   }


   // Calculate transform by decomposing the Correlation Matrix using SVD.
   Eigen::Matrix4f transform = CalculateTransformParallel(threads, correlBuffer, meanBuffer);

   // Calculate transform on the CPU.
//   int matches[cloudOne.size()/3];
//   _openCL->ReadBufferInt(matchBuffer, matches, cloudOne.size()/3);
//   std::vector<int> matchesVector(matches, matches + cloudOne.size()/3);
//   Eigen::Matrix4f transformCPU = CalculateTransformSequential(cloudOne, cloudTwo, matchesVector);
   // std::cout << "Transfrom CPU:" << std::endl << transformCPU << std::endl;

   auto end_point = std::chrono::steady_clock::now();
   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

   printf("Total Time Taken: %.3lf ms\n", duration);

   return transform;
}

Eigen::Matrix4f IterativeClosestPoint::CalculateTransformParallel(uint32_t threads, uint8_t correlBuffer, uint8_t meanBuffer)
{
   float centroidMatrixBuffer[6 * threads];
   int numMatrixBuffer[threads];
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->centroidKernel, cl::NullRange, cl::NDRange((int)threads), cl::NullRange);
   _openCL->ReadBufferFloat(meanBuffer, centroidMatrixBuffer, 6 * threads);

   Eigen::Map<Eigen::Matrix<float, 6, Eigen::Dynamic>, Eigen::RowMajor> centroids(centroidMatrixBuffer, 6, threads);
   Eigen::MatrixXf centroidMatVec = centroids.rowwise().mean();

   float correlationMatrixBuffer[9 * threads];
   _openCL->commandQueue.enqueueNDRangeKernel(_openCL->correlationKernel, cl::NullRange, cl::NDRange((int)threads), cl::NullRange);
   _openCL->ReadBufferFloat(correlBuffer, correlationMatrixBuffer, 9 * threads);

   _openCL->commandQueue.finish();

   Eigen::Map<Eigen::Matrix<float, 9, Eigen::Dynamic>, Eigen::RowMajor> correlation(correlationMatrixBuffer, 9, threads);
   Eigen::MatrixXf correlMatVec = correlation.rowwise().sum();
   correlMatVec.resize(3,3);
   correlMatVec.transposeInPlace();

   return ExtractTransform(correlMatVec, Eigen::Vector3f(centroidMatVec(0), centroidMatVec(1), centroidMatVec(2)),
                           Eigen::Vector3f(centroidMatVec(3), centroidMatVec(4), centroidMatVec(5)));
}

Eigen::Matrix4f IterativeClosestPoint::ExtractTransform(Eigen::Matrix3f correlation, Eigen::Vector3f meanOne, Eigen::Vector3f meanTwo)
{
   // Recover rotation matrix from the SVD matrices.
   Eigen::JacobiSVD<Eigen::MatrixXf> svd(correlation, Eigen::ComputeThinU | Eigen::ComputeThinV);
   Eigen::Matrix3f rotation = svd.matrixU() * svd.matrixV().transpose();

   // Interpolate using Momentum Factor and add randomness.
//   Eigen::Quaternion<float> quat1(Eigen::Matrix3f::Identity());
//   Eigen::Quaternion<float> quat2(rotation);
//   Eigen::Quaternion<float> quat3 = quat1.slerp(6.0, quat2);

   // Construct a 4x4 transformation matrix from rotation and centroidal displacement.
   Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
   transform.block<3,3>(0,0) = rotation;
   transform.block<3,1>(0,3) = meanOne - meanTwo;

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
      if(currentMatches[i] == matches[i])
      {
         count++;
      }
   }
   printf("Percent Correct Matches: %.2lf, Total Correct Matches: %d\n", 100.0 * (float)count/((float)currentMatches.size()), count);
}

Eigen::Matrix4f IterativeClosestPoint::CalculateTransformSequential(std::vector<float>& cloudOne, std::vector<float>& cloudTwo, std::vector<int>& matchesVector)
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
   // std::cout << "Full First Mean CPU: " << smallMeanOne << std::endl;
   // std::cout << "Full Second Mean CPU: " << smallMeanTwo << std::endl;

   // Calculate centroid vectors for both the pointclouds.
   Eigen::Vector3f firstMean = firstCloud.colwise().mean();
   Eigen::Vector3f secondMean = secondCloud.colwise().mean();
   // std::cout << "First Mean CPU: " << firstMean << std::endl;
   // std::cout << "Second Mean CPU: " << secondMean << std::endl;

   // Remove centroid from both pointcloud matrices.
   firstCloud = firstCloud - firstMean.rowwise().replicate(firstCloud.rows()).transpose();
   secondCloud = secondCloud - secondMean.rowwise().replicate(secondCloud.rows()).transpose();

   // Calculate the correlation matrix.
   Eigen::Matrix3f correlation = firstCloud.transpose() * secondCloud;
   // std::cout << "Correlation:" << correlation << std::endl;
   // Calculate SVD for the correlation matrix.

   return ExtractTransform(correlation, firstMean, secondMean);
}
