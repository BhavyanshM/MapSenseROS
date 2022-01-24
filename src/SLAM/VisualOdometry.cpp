//
// Created by quantum on 6/30/21.
//

#include "VisualOdometry.h"

VisualOdometry::VisualOdometry(int argc, char **argv, NetworkManager *network, ApplicationState& app, DataManager *data)
{
   _dataReceiver = network;
   _data = data;
   _bundleAdjustment = new BundleAdjustment(_data->GetLeftCamera());

   CLAY_LOG_INFO("Params: {} {} {} {}", data->GetLeftCamera()._fx, data->GetLeftCamera()._cx, data->GetLeftCamera()._fy, data->GetLeftCamera()._cy);

   cameraPose = Eigen::Matrix4f::Identity();

   // initialize values for StereoSGBM parameters
   stereo->setNumDisparities(app.STEREO_NUM_DISPARITIES * 16);
   stereo->setBlockSize(2 * app.STEREO_BLOCK_SIZE + 1);
   stereo->setPreFilterSize(2 * app.STEREO_PRE_FILTER_SIZE + 1);
   stereo->setPreFilterType(app.STEREO_PRE_FILTER_TYPE);
   stereo->setPreFilterCap(app.STEREO_PRE_FILTER_CAP);
   stereo->setMinDisparity(app.STEREO_MIN_DISPARITY);
   stereo->setTextureThreshold(app.STEREO_TEXTURE_THRESHOLD);
   stereo->setUniquenessRatio(app.STEREO_UNIQUENESS_RATIO);
   stereo->setSpeckleRange(app.STEREO_SPECKLE_RANGE);
   stereo->setSpeckleWindowSize(app.STEREO_SPECKLE_WINDOW_SIZE);
   stereo->setDisp12MaxDiff(app.STEREO_DISP_12_MAX_DIFF);
}

void VisualOdometry::ImGuiUpdate(ApplicationState& app)
{
   ImGui::Text("Stereo SGBM");
   ImGui::SliderInt("Num Disparities", &app.STEREO_NUM_DISPARITIES, 1, 4);
   ImGui::SliderInt("Block Size", &app.STEREO_BLOCK_SIZE, 2, 15);
   ImGui::SliderInt("PreFilter Size", &app.STEREO_PRE_FILTER_SIZE, 5, 40);
   stereo->setNumDisparities(app.STEREO_NUM_DISPARITIES * 16);
   stereo->setBlockSize(2 * app.STEREO_BLOCK_SIZE + 1);
   stereo->setPreFilterSize(2 * app.STEREO_PRE_FILTER_SIZE + 1);
   //   ImGui::SliderInt("PreFilter Cap", &app.STEREO_PRE_FILTER_CAP, 31, 32);
   //   ImGui::SliderInt("Min Disparity", &app.STEREO_MIN_DISPARITY);
   //   ImGui::SliderInt("Texture Threshold", &app.STEREO_TEXTURE_THRESHOLD);
   //   ImGui::SliderInt("Uniqueness Ratio", &app.STEREO_UNIQUENESS_RATIO);
   //   ImGui::SliderInt("Speckle Range", &app.STEREO_SPECKLE_RANGE);
   //   ImGui::SliderInt("Speckle Window Size", &app.STEREO_SPECKLE_WINDOW_SIZE);
   //   ImGui::SliderInt("Disp 12 Max-Diff", &app.STEREO_DISP_12_MAX_DIFF);
}

void VisualOdometry::DrawMatches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts)
{
   for (int i = 0; i < prev_pts.size(); i++)
   {
      line(img, prev_pts[i], cur_pts[i], cv::Scalar(0, 255, 0), 3);
      circle(img, prev_pts[i], 2, cv::Scalar(0, 0, 0), -1);
      circle(img, cur_pts[i], 2, cv::Scalar(255, 255, 255), -1);
   }
}

void VisualOdometry::DrawLandmarks(cv::Mat& img, std::vector<PointLandmark>& landmarks)
{
   for (int i = 0; i < landmarks.size(); i++)
   {
      cv::Point2f first(landmarks[i].GetMeasurements2D()[0].x() + _data->GetLeftCamera()._cx, landmarks[i].GetMeasurements2D()[0].y() + _data->GetLeftCamera()._cy);
      cv::Point2f second(landmarks[i].GetMeasurements2D()[1].x() + _data->GetLeftCamera()._cx, landmarks[i].GetMeasurements2D()[1].y() + _data->GetLeftCamera()._cy);

      float dist = cv::norm(first - second);
//      CLAY_LOG_INFO("DrawLandmark: First({}, {}), Second:({} {}), Dist({}), Total Measurements: {}", first.x, first.y, second.x, second.y, dist, landmarks[i].GetMeasurements2D().size());

      if(dist < 100)
      {
         line(img, first, second, cv::Scalar(0, 255, 0), 3);
         circle(img, first, 2, cv::Scalar(0, 0, 0), -1);
         circle(img, second, 2, cv::Scalar(255, 255, 255), -1);
      }

   }
}

void VisualOdometry::ExtractKeypoints_FAST(cv::Mat img_1, vector<cv::Point2f>& points1)
{
   vector<cv::KeyPoint> keypoints_1;
   int fast_threshold = 20;
   bool nonmaxSuppression = true;
   cv::FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
   cv::KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void VisualOdometry::ExtractKeypoints(cv::Mat img, cv::Ptr<cv::ORB> orb, std::vector<cv::KeyPoint>& points, cv::Mat& desc)
{
   desc.setTo(0);
   points.clear();
   orb->detectAndCompute(img, cv::noArray(), points, desc);
}


void VisualOdometry::MatchKeypoints(cv::Mat& descTrain, cv::Mat& descQuery, std::vector<cv::DMatch>& matches)
{
   matches.clear();
   using namespace cv;
   //   BFMatcher matcher(NORM_HAMMING, true);
   //   matcher.match( descQuery, descTrain, matches);

   //   std::sort(matches.begin(), matches.end(), [&](const cv::DMatch& a, const cv::DMatch& b)
   //   {
   //      return a.distance < b.distance;
   //   });
   //
   //   if(matches.size() > 1200) matches.resize(1200);

   static cv::Ptr<cv::DescriptorMatcher> bf_matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
   //   Ptr<DescriptorMatcher> flannMatcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
   std::vector<std::vector<DMatch> > knn_matches;
   bf_matcher->knnMatch(descQuery, descTrain, knn_matches, 2);
   //-- Filter matches using the Lowe's ratio test
   const float ratio_thresh = 0.8f;
   for (size_t i = 0; i < knn_matches.size(); i++)
   {
      if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
      {
         matches.push_back(knn_matches[i][0]);
      }
   }

   //   for(auto match : matches)
   //      CLAY_LOG_INFO("Match Distance: {}", match.distance);

   //-- Filter matches
   //   std::vector<cv::DMatch> finalMatches;
   //   for (size_t i = 0; i < matches.size(); i++)
   //   {
   //         if(matches[i].distance < 40)
   //            finalMatches.push_back(matches[i]);
   //   }
   //   matches = finalMatches

   //   CLAY_LOG_INFO("MatchKeypoints(): Total Matches: {}", matches.size());

}

void VisualOdometry::GridSampleKeypoints(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::DMatch>& matches)
{
   int xStep = width / xGridCount;
   int yStep = height / yGridCount;

   bool grid[yGridCount][xGridCount];
   memset(grid, false, sizeof(bool) * yGridCount * xGridCount);

   std::vector<cv::DMatch> finalKeypoints;
   for (int i = 0; i < matches.size(); i++)
   {
      cv::Point2f point(keypoints[matches[i].trainIdx].pt);

      if (point.x >= 0 && point.x < width && point.y >= 0 && point.y < height)
      {
         int xIndex = (int) ((float) point.x / (float) xStep);
         int yIndex = (int) ((float) point.y / (float) yStep);
         //   CLAY_LOG_INFO("i: {}, Size: {} Dims:{} {} Point: {} {} -> {} {}", i, matches.size(), width, height, point.x, point.y, xIndex, yIndex);

         if (xIndex >= 0 && xIndex < xGridCount && yIndex >= 0 && yIndex < yGridCount)
         {
            if (!grid[yIndex][xIndex])
            {
               finalKeypoints.push_back(matches[i]);
               grid[yIndex][xIndex] = true;
            }
         }
      }
   }

   matches.clear();
   matches = finalKeypoints;
}



void VisualOdometry::TriangulateStereoNormal(std::vector<cv::KeyPoint>& pointsTrain, std::vector<cv::KeyPoint>& pointsQuery, std::vector<cv::DMatch>& matches,
                                             std::vector<PointLandmark>& points3D)
{
   points3D.clear();
   float x1, x2, y1, y2, y_hat, X, Y, Z;
   float cx = 607.1928, cy = 185.2157;
   for (auto match: matches)
   {
      //      CLAY_LOG_INFO("{} {}", pointsTrain[match.trainIdx].pt.x, pointsQuery[match.queryIdx].pt.x);

      /*
       * Left = 1, Right = 2
       * Z = f * B / (x1 - x2)
       * X = Z / (x1 * f) = x1 * B / (x1 - x2)
       * Y = X * (y_hat) / x1 = (y_hat / 2) * (B / (x1 - x2))
       * */

      x1 = pointsTrain[match.trainIdx].pt.x - cx;
      x2 = pointsQuery[match.queryIdx].pt.x - cx;
      y1 = pointsTrain[match.trainIdx].pt.y - cy;
      y2 = pointsQuery[match.queryIdx].pt.y - cy;

      if (abs(x1 - x2) > 1.0f && abs(y1 - y2) < 10.0f)
      {
         y_hat = (y1 + y2) / 2;

         Z = _data->GetLeftCamera()._fx * _data->GetStereoBaseline() / (x1 - x2);
         X = x1 * _data->GetStereoBaseline() / (x1 - x2);
         Y = y_hat * (_data->GetStereoBaseline() / (x1 - x2));

         //         CLAY_LOG_INFO("Point3D: {} {} {}", X, Y, Z);
         if (Z > 0)
         {
            Eigen::Vector3f point(X, Y, Z);

            PointLandmark landmark(point);
            Eigen::Vector2f measurement(x1, y1);
            landmark.AddMeasurement2D(measurement, match.trainIdx, 0);

            points3D.emplace_back(landmark);
         }
      }
   }
   //   CLAY_LOG_INFO("Triangulate(): Total Depth Points: {}", points3D.size());
}

void VisualOdometry::ExtractFinalSet(std::vector<cv::DMatch> leftMatches, std::vector<cv::KeyPoint> curLeftKp, std::vector<PointLandmark>& points3D)
{
   int overlap = 0;
   for (auto match: leftMatches)
   {
      for (int i = 0; i < points3D.size(); i++)
      {
         if (points3D[i]._index[0] == match.trainIdx)
         {
            Eigen::Vector2f measurement(curLeftKp[match.queryIdx].pt.x - _data->GetLeftCamera()._cx,
                            curLeftKp[match.queryIdx].pt.y - _data->GetLeftCamera()._cy);

            Eigen::Vector2f prevMeasurement = points3D[i].GetMeasurements2D()[0];
            Eigen::Vector2f oneMeasurement = points3D[i].GetMeasurements2D()[1];

//            CLAY_LOG_INFO("Match: PrevStereoPoint({}, {})", measurement.x(), measurement.y());
//            CLAY_LOG_INFO("Match: Zero({}, {}), One:({} {})", prevMeasurement.x(), prevMeasurement.y(), oneMeasurement.x(), oneMeasurement.y());

            points3D[i].AddMeasurement2D(measurement, match.queryIdx, 1);
         }
      }

   }

   for(int i = points3D.size() - 1; i>=0; i--)
   {
      if(points3D[i].GetMeasurements2D().size() != 2)
      {
         points3D.erase(points3D.begin() + i);
      }
   }
   CLAY_LOG_INFO("Total Overlap PointLandmarks: {}", points3D.size());
}



cv::Mat
VisualOdometry::EstimateMotion_2D2D(std::vector<cv::Point2f>& prevFeatures, std::vector<cv::Point2f>& curFeatures, cv::Mat& mask, const CameraParams& cam)
{
   using namespace cv;
   float data[9] = {cam._fx, 0, cam._cx, 0, cam._fy, cam._cy, 0, 0, 1};
   cv::Mat K = cv::Mat(3, 3, CV_32FC1, data);

   cv::Mat R(3, 3, CV_32FC1), t(1, 3, CV_32FC1);
   cv::Mat E = findEssentialMat(prevFeatures, curFeatures, K, cv::RANSAC, 0.999, 1.0, mask);
   recoverPose(E, prevFeatures, curFeatures, K, R, t, mask);

   CLAY_LOG_INFO("Features: {} {} {} {}", prevFeatures.size(), curFeatures.size(), mask.rows, mask.cols);
   for (int i = prevFeatures.size() - 1; i >= 0; i--)
   {
      if ((int) mask.at<unsigned char>(i, 0) == 1)
      {
         prevFeatures.erase(prevFeatures.begin() + i);
         curFeatures.erase(curFeatures.begin() + i);
      }
   }
   CLAY_LOG_INFO("Inliers: {} {}", prevFeatures.size(), curFeatures.size());

   Mat cvPose = Mat::eye(4, 4, CV_32FC1);
   R.copyTo(cvPose(Range(0, 3), Range(0, 3))); /* Excludes the 'end' element */
   t.copyTo(cvPose(Range(0, 3), Range(3, 4)));
   cv::invert(cvPose, cvPose);
   return cvPose;
}



cv::Mat VisualOdometry::CalculateStereoDepth(cv::Mat left, cv::Mat right)
{
   cv::Mat disparity;
   stereo->compute(left, right, disparity);
   disparity.convertTo(disparity, CV_8U, 1.0);
}

void VisualOdometry::LoadImages(ApplicationState& appState)
{
   double timestamp = 0;
   if (appState.DATASET_ENABLED)
   {
      leftImage = _data->GetNextImage();
      rightImage = _data->GetNextSecondImage();
   } else if (_dataReceiver)
   {
      ImageReceiver *left = (ImageReceiver *) this->_dataReceiver->receivers[appState.L515_COLOR];
      ImageReceiver *right = (ImageReceiver *) this->_dataReceiver->receivers[appState.L515_COLOR];

      left->getData(leftImage, appState, timestamp);
      right->getData(rightImage, appState, timestamp);
   }
}

void VisualOdometry::CalculateOdometry_ORB(ApplicationState& appState, Keyframe& kf, cv::Mat leftImage, cv::Mat rightImage, cv::Mat& cvPose, std::vector<PointLandmark>& points3D)
{
   //   TriangulateStereoNormal(kp_prevLeft, kp_prevRight, prevMatchesStereo, _prevPoints3D, 0.54, 718.856);

   ExtractKeypoints(leftImage, orb, kp_curLeft, desc_curLeft);
   ExtractKeypoints(rightImage, orb, kp_curRight, desc_curRight);
   MatchKeypoints(kf.descLeft, desc_curLeft, matchesLeft);
   MatchKeypoints(kf.descLeft, kf.descRight, prevMatchesStereo);

   CLAY_LOG_INFO("Stereo Matches: {} {} {}", prevMatchesStereo.size(), kf.descLeft.rows, kf.descRight.rows);
//   cv::drawMatches(kf.rightImage, kf.keypointsRight, kf.leftImage, kf.keypointsLeft, prevMatchesStereo, prevFinalDisplay);

   for (int i = matchesLeft.size() - 1; i >= 0; i--)
   {
      auto m = matchesLeft[i];
      float dist = cv::norm(kp_curLeft[m.queryIdx].pt - kf.keypointsLeft[m.trainIdx].pt);
      if (dist > 50.0f)
      {
         matchesLeft.erase(matchesLeft.begin() + i);
      }
   }

   for (int i = prevMatchesStereo.size() - 1; i >= 0; i--)
   {
      auto m = prevMatchesStereo[i];
      int dist = kp_curLeft[m.queryIdx].pt.y - kf.keypointsLeft[m.trainIdx].pt.y;
      if (dist > 8)
      {
         prevMatchesStereo.erase(prevMatchesStereo.begin() + i);
      }
   }

   CLAY_LOG_INFO("Points To Be Triangulated: {}", prevMatchesStereo.size());

   TriangulateStereoNormal(kf.keypointsLeft, kf.keypointsRight, prevMatchesStereo, points3D);
   CLAY_LOG_INFO("Points Triangulated: {}", points3D.size());

   prevPoints2D.clear();
   curPoints2D.clear();
   for (auto m: matchesLeft)
   {
      prevPoints2D.emplace_back(cv::Point2f(kf.keypointsLeft[m.trainIdx].pt));
      curPoints2D.emplace_back(cv::Point2f(kp_curLeft[m.queryIdx].pt));
   }

   CLAY_LOG_INFO("Points for Motion Estimation: {} {}", prevPoints2D.size(), curPoints2D.size());

   cv::Mat mask, pose;
   if (prevPoints2D.size() >= 10 && curPoints2D.size() >= 10)
   {
      pose = EstimateMotion_2D2D(prevPoints2D, curPoints2D, mask, _data->GetLeftCamera());
      cvPose = pose;

      /* Triangulate Feature Points */
//      if (prevPoints2D.size() > 10)
//      {
//         cv::Mat points = TriangulatePoints(prevPoints2D, curPoints2D, _data->GetLeftCamera(), pose);
//         points4D = points;
//      }
   }

   ExtractFinalSet(matchesLeft, kp_curLeft, points3D);

   DrawLandmarks(prevFinalDisplay, points3D);



//   for(auto point : points3D)
//   {
//      CLAY_LOG_INFO("Landmark: {} {} {}", point.GetMeasurements2D()[0].y(), point.GetMeasurements2D()[1].y(), point.GetMeasurements2D()[2].y());
//   }




   //   printf("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\n",
   //          cvPose.at<float>(0), cvPose.at<float>(1), cvPose.at<float>(2), cvPose.at<float>(3),
   //          cvPose.at<float>(4), cvPose.at<float>(5), cvPose.at<float>(6), cvPose.at<float>(7),
   //          cvPose.at<float>(8), cvPose.at<float>(9), cvPose.at<float>(10), cvPose.at<float>(11));



   //   cv::drawMatches(curLeft, kp_curLeft, kf.image, kf.keypoints, matchesLeft, prevFinalDisplay);

   prevLeft = curLeft.clone();
   desc_prevLeft = desc_curLeft.clone();
   kp_prevLeft = kp_curLeft;

   prevRight = curRight.clone();
   desc_prevRight = desc_curRight.clone();
   kp_prevRight = kp_curRight;

   count++;

//   prevFinalDisplay = leftImage;
}

bool VisualOdometry::Update(ApplicationState& appState, Clay::Ref<Clay::TriangleMesh> axes, Clay::Ref<Clay::PointCloud> cloud)
{
   auto start_point = std::chrono::steady_clock::now();
   cv::Mat cvPose, points4D;
   std::vector<PointLandmark> points3D;

   LoadImages(appState);
   if (!leftImage.empty() && leftImage.rows > 0 && leftImage.cols > 0 && !rightImage.empty() && rightImage.rows > 0 && rightImage.cols > 0)
   {
      /* During first iteration, simply store (prev) grayscale images, extract keypoints,
       * and insert the first keyframe at identity pose, and return false, since no pose was computed. */
      if (count == 0)
      {
         width = leftImage.cols;
         height = leftImage.rows;
         cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
         cvtColor(rightImage, prevRight, cv::COLOR_BGR2GRAY);
         ExtractKeypoints(prevLeft, orb, kp_prevLeft, desc_prevLeft);
         ExtractKeypoints(prevRight, orb, kp_prevRight, desc_prevRight);
         _keyframes.emplace_back(Keyframe(desc_prevLeft.clone(), desc_prevRight.clone(), kp_prevLeft, kp_prevRight, Eigen::Matrix4f::Identity(), leftImage.clone(), rightImage.clone()));
         count++;
         return false;
      }

      /* From second iteration onwards, store (cur) grayscale images, initialize, and insert keyframes when needed. */
      cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
      cvtColor(rightImage, curRight, cv::COLOR_BGR2GRAY);

      /* Initialization step: Build initial local map of 3D points; Insert the second keyframe. */
      if (!_initialized)
      {
         auto kf = _keyframes[0];
         CalculateOdometry_ORB(appState, kf, curLeft, curRight, cvPose, points3D);

         Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigenPose(reinterpret_cast<float *>(cvPose.data));
         eigenPose.transposeInPlace();
         cameraPose = cameraPose * eigenPose;

//         prevFinalDisplay = leftImage.clone();
//         DrawMatches(prevFinalDisplay, prevPoints2D, curPoints2D);

         if (cameraPose.block<3, 1>(0, 3).norm() > 1)
         {
            _initialized = true;
            _keyframes.emplace_back(Keyframe(desc_curLeft.clone(), desc_curRight.clone(),
                                             kp_curLeft, kp_curRight, cameraPose, leftImage.clone(), rightImage.clone()));
         }
      } else
      {
         prevFinalDisplay = leftImage.clone();

         auto kf = _keyframes[_keyframes.size() - 1];
         CalculateOdometry_ORB(appState, kf, curLeft, curRight, cvPose, points3D);
         cvCurPose = cvCurPose * cvPose;

         Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigenPose(reinterpret_cast<float *>(cvPose.data));
         eigenPose.transposeInPlace();
         cameraPose = cameraPose * eigenPose;

//         DrawMatches(prevFinalDisplay, prevPoints2D, curPoints2D);

         if (eigenPose.block<3, 1>(0, 3).norm() > 0.8)
         {
            _initialized = true;
            _keyframes.emplace_back(
                  Keyframe(desc_curLeft.clone(), desc_curRight.clone(), kp_curLeft, kp_curRight, cameraPose, leftImage.clone(), rightImage.clone()));

            CLAY_LOG_INFO("Performing Bundle Adjustment.");

            /* ------------------------- BUNDLE ADJUSTMENT ------------------------------*/
//            std::vector<Eigen::Matrix4f> poses;
//            poses.emplace_back(_keyframes[_keyframes.size() - 2].pose);
//            poses.emplace_back(_keyframes[_keyframes.size() - 1].pose);
//            _bundleAdjustment->Update(points3D, poses);
            /* ------------------------- BUNDLE ADJUSTMENT ------------------------------*/

            _bundleAdjustment->Optimize();


            if (axes)
            {
               glm::mat4 glmTransform;
               for (int i = 0; i < 4; ++i)
                  for (int j = 0; j < 4; ++j)
                     glmTransform[j][i] = cameraPose(i, j);
               glmTransform[3][0] *= scalar;
               glmTransform[3][1] *= scalar;
               glmTransform[3][2] *= scalar;
               axes->ApplyTransform(glmTransform);
            }

            /* Triangulated Points */
            /* TODO: Filter points by 5-point algorithm mask before triangulation. */
            if (cloud)
            {
               for (int i = 0; i < points3D.size(); i++)
               {
                  if (i % 4 == 0)
                  {
                     Eigen::Vector4f point;
                     point << points3D[i]._point3D, 1;
                     point = cameraPose * point;
                     cloud->InsertVertex(scalar * point.x() / point.w(), scalar * point.y() / point.w(), scalar * point.z() / point.w());
                  }
               }
            }
         }
      }
   }

   auto end_point = std::chrono::steady_clock::now();
   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

   //   CLAY_LOG_INFO("(Visual Odometry) Total Time Taken: {} ms\n", duration);

   return true;
}

void VisualOdometry::Show()
{
   if (!prevFinalDisplay.empty() && prevFinalDisplay.rows != 0 && prevFinalDisplay.cols != 0)
      cv::imshow("Previous Keypoints Visualizer", prevFinalDisplay);
   if (!curFinalDisplay.empty() && curFinalDisplay.rows != 0 && curFinalDisplay.cols != 0)
      cv::imshow("Keypoints Visualizer", curFinalDisplay);
   cv::waitKey(1);
}
