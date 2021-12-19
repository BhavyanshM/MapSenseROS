//
// Created by quantum on 6/30/21.
//

#include "VisualOdometry.h"

VisualOdometry::VisualOdometry(int argc, char **argv, NetworkManager *network, ApplicationState& app, DataManager *data) : _dataReceiver(network), _data(data)
{
   cameraPose = Eigen::Matrix4f::Identity();

   // initialize values for StereoSGBM parameters
   stereo->setNumDisparities( app.STEREO_NUM_DISPARITIES * 16);
   stereo->setBlockSize( 2 * app.STEREO_BLOCK_SIZE + 1);
   stereo->setPreFilterSize( 2 * app.STEREO_PRE_FILTER_SIZE + 1);
   stereo->setPreFilterType( app.STEREO_PRE_FILTER_TYPE);
   stereo->setPreFilterCap( app.STEREO_PRE_FILTER_CAP);
   stereo->setMinDisparity( app.STEREO_MIN_DISPARITY);
   stereo->setTextureThreshold( app.STEREO_TEXTURE_THRESHOLD);
   stereo->setUniquenessRatio( app.STEREO_UNIQUENESS_RATIO);
   stereo->setSpeckleRange( app.STEREO_SPECKLE_RANGE);
   stereo->setSpeckleWindowSize( app.STEREO_SPECKLE_WINDOW_SIZE);
   stereo->setDisp12MaxDiff( app.STEREO_DISP_12_MAX_DIFF);
}

void VisualOdometry::ImGuiUpdate(ApplicationState& app)
{
   ImGui::Text("Stereo SGBM");
   ImGui::SliderInt("Num Disparities", &app.STEREO_NUM_DISPARITIES, 1, 4);
   ImGui::SliderInt("Block Size", &app.STEREO_BLOCK_SIZE, 2, 15);
   ImGui::SliderInt("PreFilter Size", &app.STEREO_PRE_FILTER_SIZE, 5, 40);
   stereo->setNumDisparities( app.STEREO_NUM_DISPARITIES * 16);
   stereo->setBlockSize( 2 * app.STEREO_BLOCK_SIZE + 1);
   stereo->setPreFilterSize( 2 * app.STEREO_PRE_FILTER_SIZE + 1);
//   ImGui::SliderInt("PreFilter Cap", &app.STEREO_PRE_FILTER_CAP, 31, 32);
//   ImGui::SliderInt("Min Disparity", &app.STEREO_MIN_DISPARITY);
//   ImGui::SliderInt("Texture Threshold", &app.STEREO_TEXTURE_THRESHOLD);
//   ImGui::SliderInt("Uniqueness Ratio", &app.STEREO_UNIQUENESS_RATIO);
//   ImGui::SliderInt("Speckle Range", &app.STEREO_SPECKLE_RANGE);
//   ImGui::SliderInt("Speckle Window Size", &app.STEREO_SPECKLE_WINDOW_SIZE);
//   ImGui::SliderInt("Disp 12 Max-Diff", &app.STEREO_DISP_12_MAX_DIFF);
}

void VisualOdometry::DrawMatches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts){
   for (int i = 0; i<prev_pts.size(); i++){
      line(img, prev_pts[i],  cur_pts[i], cv::Scalar(0,255,0), 3);
      circle(img, prev_pts[i], 2, cv::Scalar(0,0,0), -1);
      circle(img,  cur_pts[i], 2, cv::Scalar(255, 255, 255), -1);
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
   orb->detectAndCompute(img,  cv::noArray(), points, desc);
//   if(desc.type() != CV_32F)
//   {
//      desc.convertTo(desc, CV_32F);
//   }
//   CLAY_LOG_INFO("ExtractKeypoints(): Total Keypoint Features: {}", points.size());
}

void VisualOdometry::TrackKeypoints(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts)
{
   std::vector<uchar> status;
   std::vector<float> err;
   cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);

   cur_pts.clear();
   calcOpticalFlowPyrLK(prev, cur, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3, criteria);

   ROS_DEBUG("Previous Keypoints: (%d) \t Current Keypoints: (%d)", prev_pts.size(), cur_pts.size());

   int lastIndex = 0;
   for (uint i = 0; i < prev_pts.size(); i++)
   {
      // Select good points
      cv::Point2f point = cur_pts.at(i - lastIndex);
      if ((status.at(i) == 0)||(point.x<0)||(point.y<0))	{
         prev_pts.erase(prev_pts.begin() + i);
         cur_pts.erase(cur_pts.begin() + i);
         lastIndex++;
      }
      else
      if((status.at(i) == 1))
      {
         float dist = norm((cur_pts[i] - prev_pts[i]));
         if(err[i] > 3.0f || dist > 20)
         {
            prev_pts.erase(prev_pts.begin() + i);
            cur_pts.erase(cur_pts.begin() + i);
            lastIndex++;
         }
      }
      // else{
      //     printf("(%d)(%.2lf, %.2lf) -> (%.2lf, %.2lf)\n", i, prev_pts[i].x, prev_pts[i].y,cur_pts[i].x,cur_pts[i].y);
      // }
   }
}

void VisualOdometry::DrawAllMatches(cv::Mat& image)
{
   for (int i = 0; i < matchesLeft.size(); i++)
   {
//      cv::circle(leftImage, kp_prevLeft[matchesLeft[i].queryIdx].pt, 4, cv::Scalar(255, 255, 0), -1);
      cv::circle(leftImage, kp_curLeft[matchesLeft[i].trainIdx].pt, 4, cv::Scalar(0, 255, 255), -1);
//      cv::line(leftImage, kp_prevLeft[matchesLeft[i].queryIdx].pt, kp_curLeft[matchesLeft[i].trainIdx].pt, cv::Scalar(0, 255, 0), 2);
   }
   for (int i = 0; i < matchesRight.size(); i++)
   {
//      cv::circle(rightImage, kp_prevRight[matchesRight[i].queryIdx].pt, 4, cv::Scalar(255, 255, 0), -1);
      cv::circle(rightImage, kp_curRight[matchesRight[i].trainIdx].pt, 4, cv::Scalar(0, 255, 255), -1);
//      cv::line(rightImage, kp_prevRight[matchesRight[i].queryIdx].pt, kp_curRight[matchesRight[i].trainIdx].pt, cv::Scalar(0, 255, 0), 2);
   }
   std::vector<cv::Mat> images;
   images.emplace_back(leftImage);
   images.emplace_back(rightImage);
   cv::Mat combined;
   cv::hconcat(images, combined);
   cv::Point2f dims(leftImage.cols, 0);
//   for (int i = 0; i < prevMatchesStereo.size(); i++)
//   {
//      cv::line(combined, kp_curLeft[prevMatchesStereo[i].queryIdx].pt, kp_curRight[prevMatchesStereo[i].trainIdx].pt + dims, cv::Scalar(255, 150, 0), 1);
////      cv::circle(combined, prev_pts[i], 2, cv::Scalar(0, 0, 0), -1);
////      cv::circle(combined, cur_pts[i], 2, cv::Scalar(255, 255, 255), -1);
//   }
   image = combined;
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
   std::vector< std::vector<DMatch> > knn_matches;
   bf_matcher->knnMatch( descQuery, descTrain, knn_matches, 2 );
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

void VisualOdometry::TriangulateStereoPoints(cv::Mat& leftPoseWorld, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, std::vector<cv::DMatch> stereoMatches,
                                             std::vector<PointLandmark> points3D)
{
   /* Calculate Essential Matrix from Correspondences and Intrinsics
       (TUM-RGBD) - fx:517.3 fy:516.5 cx:318.6 cy:255.3
       (KITTI) - fx:718.856 fy:718.856 cx:607.193 cy:185.216
       (IHMC-Chest-L515) - fx:602.259 fy:603.040 cx:321.375 cy:240.515
   */
   using namespace cv;

   std::vector<Point2f> leftPoints2D;
   std::vector<Point2f> rightPoints2D;
   for(auto match : stereoMatches) { leftPoints2D.push_back(kpLeft[match.queryIdx].pt); rightPoints2D.push_back(kpRight[match.trainIdx].pt); }

   float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216;
   float data[9] = {   fx, 0, cx, 0, fy, cy, 0, 0, 1 };
   cv::Mat K = cv::Mat(3, 3, CV_32FC1, data);
   cv::Mat R(3,3,CV_32FC1), t(1,3,CV_32FC1), mask;

   Mat Rt0 = Mat::eye(3,4,CV_32FC1);
   Mat Rt1 = Mat::eye(3,4,CV_32FC1);
   Rt1.at<float>(0,3) = 1;

//   R.copyTo(cvPose(Range(0,3), Range(0,3))); /* Excludes the 'end' element */
//   t.copyTo(cvPose(Range(0,3), Range(3,4)));


   Mat points4D;
   cv::triangulatePoints(K * Rt0, K * Rt1,
                         leftPoints2D, rightPoints2D,
                         points4D);

   Mat combined;
   drawMatches(curLeft, kp_curLeft, curRight, kp_curRight, prevMatchesStereo, combined);
   imshow("Matches", combined);
   waitKey(1);

   int count = 0;
   for(int m = 0; m<points4D.cols; m++){
      Mat point = points4D.col(m) / points4D.at<float>(3,m);
      float z = point.at<float>(2);
//      CLAY_LOG_INFO("Triangulated Point: {} {} {}", point.at<float>(0,m), point.at<float>(1,m), point.at<float>(2,m));
      if (z > 0 && z < 1000) {
         points3D.emplace_back(
               PointLandmark( Vector3f(point.at<float>(0,m), point.at<float>(1,m), point.at<float>(2,m))));
         count++;
      }
   }
//   CLAY_LOG_INFO("After Triangulation: {} {} {} {} {}", leftPoints2D.size(), rightPoints2D.size(), points4D.rows, points4D.cols, points3D.size());

}

void VisualOdometry::TriangulateStereoNormal(std::vector<cv::KeyPoint>& pointsTrain, std::vector<cv::KeyPoint>& pointsQuery, std::vector<cv::DMatch>& matches, std::vector<PointLandmark>& points3D, float baseline, float focalLength)
{
   points3D.clear();
   float x1, x2, y1, y2, y_hat, X, Y, Z;
   float cx = 607.1928, cy = 185.2157;
   for(auto match : matches)
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

      if(abs(x1 - x2) > 1.0f && abs(y1 - y2) < 10.0f)
      {
         y_hat = (y1 + y2) / 2;

         Z = focalLength * baseline / (x1 - x2);
         X = x1 * baseline / (x1 - x2);
         Y = (y_hat / 2) * (baseline / (x1 - x2));

//         CLAY_LOG_INFO("Point3D: {} {} {}", X, Y, Z);
         if(Z > 0)
         {
            PointLandmark landmark(Eigen::Vector3f(X,Y,Z));
            landmark.AddMeasurement2D(Eigen::Vector2f(x1, y1), match.trainIdx, 0);
            landmark.AddMeasurement2D(Eigen::Vector2f(x2, y2), match.queryIdx, 1);
            points3D.emplace_back(landmark);
//            CLAY_LOG_INFO("Total Points: {} \t Points2D: [({}, {}), ({}, {})] \t Point3D: ({} {} {})", points3D.size(), x1, y1, x2, y2, X, Y, Z);
         }

      }

   }
//   CLAY_LOG_INFO("Triangulate(): Total Depth Points: {}", points3D.size());
}

void VisualOdometry::ExtractFinalSet(std::vector<cv::DMatch> leftMatches, std::vector<cv::KeyPoint> curLeftKp, std::vector<PointLandmark>& points3D)
{
   int overlap = 0;
   for(auto match : leftMatches)
   {
      for(int i = 0; i<points3D.size(); i++)
      {
         if(points3D[i]._index[0] == match.trainIdx)
         {
//            CLAY_LOG_INFO("LeftTrain:{} -> 3DTrain:{}", match.trainIdx, points3D[i]._index[0]);
            points3D[i].AddMeasurement2D(Eigen::Vector2f(curLeftKp[match.queryIdx].pt.x, curLeftKp[match.queryIdx].pt.y), match.queryIdx, 2);
            overlap++;
         }
      }
   }
//   CLAY_LOG_INFO("ExtractFinalSet(): Total Overlap PointLandmarks: {}", overlap);
}

const Eigen::Matrix4f& VisualOdometry::EstimateMotion(std::vector<PointLandmark> points, int cameraID)
{
   float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216;
   float data[9] = {   fx, 0, cx, 0, fy, cy, 0, 0, 1 };
   cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, data);

   // Read points
   std::vector<cv::Point2f> imagePoints;
   std::vector<cv::Point3f> objectPoints;

   for(auto landmark : points)
   {
      imagePoints.emplace_back(cv::Point2f(landmark.GetMeasurements2D()[2].x(), landmark.GetMeasurements2D()[2].y()));
      objectPoints.emplace_back(cv::Point3f(landmark.GetPoint3D().x(), landmark.GetPoint3D().y(), landmark.GetPoint3D().z()));
   }

//   std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;

//   std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;

   cv::Mat distCoeffs(4,1,cv::DataType<double>::type);
   distCoeffs.at<double>(0) = 0;
   distCoeffs.at<double>(1) = 0;
   distCoeffs.at<double>(2) = 0;
   distCoeffs.at<double>(3) = 0;

   cv::Mat rvec(3,1,cv::DataType<double>::type);
   cv::Mat tvec(3,1,cv::DataType<double>::type);
   cv::Mat rotation(3,3, CV_32F);

   cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false);
   cv::Rodrigues(rvec, rotation);

   cv::Mat cvPose = cv::Mat::eye(4,4,CV_32FC1);
   rotation.copyTo(cvPose(cv::Range(0,3), cv::Range(0,3))); /* Excludes the 'end' element */
   tvec.copyTo(cvPose(cv::Range(0,3), cv::Range(3,4)));

   cv::invert(cvPose, cvPose);
   this->cvCurPose = (this->cvCurPose * cvPose);

//   std::cout << "Pose: " << std::endl << cvPose << std::endl;

   printf("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\n",
          cvPose.at<float>(0), cvPose.at<float>(1), cvPose.at<float>(2), cvPose.at<float>(3),
          cvPose.at<float>(4), cvPose.at<float>(5), cvPose.at<float>(6), cvPose.at<float>(7),
          cvPose.at<float>(8), cvPose.at<float>(9), cvPose.at<float>(10), cvPose.at<float>(11));

   return Eigen::Matrix4f::Identity();

}

void VisualOdometry::Initialize(Clay::Ref<Clay::PointCloud>& cloud)
{
   std::vector<PointLandmark> prevPoints3D, curPoints3D;

   /* To Skip a Few Images. */
   for(int i = 0; i<110; i++)
   {
      _data->GetNextImage();
      _data->GetNextSecondImage();
   }

   leftImage = _data->GetNextImage();
   rightImage = _data->GetNextSecondImage();
   width = leftImage.cols;
   height = leftImage.rows;

   auto start_point = std::chrono::steady_clock::now();

   cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
   cvtColor(rightImage, prevRight, cv::COLOR_BGR2GRAY);
   ExtractKeypoints(prevLeft, orb, kp_prevLeft, desc_prevLeft);
   ExtractKeypoints(prevRight, orb, kp_prevRight, desc_prevRight);
   MatchKeypoints(desc_prevLeft, desc_prevRight, prevMatchesStereo);
   TriangulateStereoNormal(kp_prevLeft, kp_prevRight, prevMatchesStereo, prevPoints3D, 0.54, 718.856);

   leftImage = _data->GetNextImage();
   rightImage = _data->GetNextSecondImage();
   cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
   ExtractKeypoints(curLeft, orb, kp_curLeft, desc_curLeft);

   MatchKeypoints(desc_prevLeft, desc_curLeft, matchesLeft);
   ExtractFinalSet(matchesLeft, kp_curLeft, prevPoints3D);

   Eigen::Matrix4f relativeCurPose = EstimateMotion(prevPoints3D, 2);

   auto end_point = std::chrono::steady_clock::now();
   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

//   CLAY_LOG_INFO("(Visual Odometry) Total Time Taken: {} ms\n", duration);

   //   cv::imshow("Initialize Visual Odometry", curFinalDisplay);

   for(int i = 0; i<prevPoints3D.size(); i++)
   {
      cloud->InsertVertex(prevPoints3D[i].GetPoint3D().x() * 0.01, prevPoints3D[i].GetPoint3D().y() * 0.01, prevPoints3D[i].GetPoint3D().z() * 0.01);
   }

   cv::drawMatches(prevRight, kp_prevRight, prevLeft, kp_prevLeft, prevMatchesStereo, prevFinalDisplay);
   cv::drawMatches(curLeft, kp_curLeft, prevLeft, kp_prevLeft, matchesLeft, curFinalDisplay);



}

void VisualOdometry::Show()
{
   if(!prevFinalDisplay.empty() && prevFinalDisplay.rows != 0 && prevFinalDisplay.cols != 0)
      cv::imshow("Previous Keypoints Visualizer", prevFinalDisplay);
   if(!curFinalDisplay.empty() && curFinalDisplay.rows != 0 && curFinalDisplay.cols != 0)
      cv::imshow("Keypoints Visualizer", curFinalDisplay);
   cv::waitKey(1);
}

cv::Mat VisualOdometry::EstimateMotion_2D2D(std::vector<cv::Point2f>& prevFeatures, std::vector<cv::Point2f>& curFeatures, cv::Mat& mask)
{
   using namespace cv;
   float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216;
   float data[9] = {   fx, 0, cx, 0, fy, cy, 0, 0, 1 };
   cv::Mat K = cv::Mat(3, 3, CV_32FC1, data);
   cv::Mat R(3,3,CV_32FC1), t(1,3,CV_32FC1);
   cv::Mat E = findEssentialMat(prevFeatures, curFeatures, K, cv::RANSAC, 0.999, 1.0, mask);
   recoverPose(E, prevFeatures, curFeatures, K, R, t, mask);

   Mat cvPose = Mat::eye(4, 4, CV_32FC1);
   R.copyTo(cvPose(Range(0, 3), Range(0, 3))); /* Excludes the 'end' element */
   t.copyTo(cvPose(Range(0, 3), Range(3, 4)));
   cv::invert(cvPose, cvPose);
   return cvPose;

}

void VisualOdometry::CalculateOdometry_FAST(ApplicationState& appState, Eigen::Matrix4f& transform)
{
   double timestamp = 0;
   using namespace cv;
   /* Calculate Essential Matrix from Correspondences and Intrinsics
       (TUM-RGBD) - fx:517.3 fy:516.5 cx:318.6 cy:255.3
       (KITTI) - fx:718.856 fy:718.856 cx:607.193 cy:185.216
       (IHMC-Chest-L515) - fx:602.259 fy:603.040 cx:321.375 cy:240.515

        ZED2 Parameters:
            height: 720 width: 1280
            distortion_model: "plumb_bob"
            K: fx: 526.1423950195312, cx: 632.4866943359375, fy: 526.1423950195312, cy: 362.7293395996094
   */


   if (appState.DATASET_ENABLED)
   {
      leftImage = _data->GetNextImage();
      rightImage = _data->GetNextSecondImage();
   } else
   {
      ((ImageReceiver *) this->_dataReceiver->receivers[appState.KITTI_LEFT_IMG_RECT])->getData(leftImage, appState, timestamp);
      ((ImageReceiver *) this->_dataReceiver->receivers[appState.KITTI_RIGHT_IMG_RECT])->getData(rightImage, appState, timestamp);
   }

   if (!leftImage.empty() && leftImage.rows > 0 && leftImage.cols > 0 && !rightImage.empty() && rightImage.rows > 0 && rightImage.cols > 0)
   {
      std::vector<cv::Point2f> points1, points2;
      if (count == 0)
      {
         width = leftImage.cols;
         height = leftImage.rows;
         cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
         ExtractKeypoints_FAST(prevLeft, prevFeaturesLeft);
         count++;
         return;
      }

//      CLAY_LOG_INFO("Features: {}", prevFeaturesLeft.size());

      cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
      TrackKeypoints(prevLeft, curLeft, prevFeaturesLeft, curFeaturesLeft);

      DrawMatches(leftImage, prevFeaturesLeft, curFeaturesLeft);

      cv::Mat mask;
      cv::Mat cvPose = EstimateMotion_2D2D(prevFeaturesLeft, curFeaturesLeft, mask);

      this->cvCurPose = (this->cvCurPose * cvPose);

      printf("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\n",
             cvPose.at<float>(0), cvPose.at<float>(1), cvPose.at<float>(2), cvPose.at<float>(3),
             cvPose.at<float>(4), cvPose.at<float>(5), cvPose.at<float>(6), cvPose.at<float>(7),
             cvPose.at<float>(8), cvPose.at<float>(9), cvPose.at<float>(10), cvPose.at<float>(11));

      Eigen::Matrix4f cameraPose = Eigen::Matrix4f::Identity();
      for(int i = 0; i<3; i++)
         for (int j = 0; j<4; j++)
            cameraPose(i,j) = cvPose.at<float>(i,j);

      /*
       * Find inlier points from 5-point algorithm RANSAC mask.
       * */
//      vector <Point2f> trip_prev, trip_cur;
//      for (int i = 0; i < mask.rows; i++)
//      {
//         if (!mask.at<unsigned char>(i))
//         {
//            trip_prev.push_back(cv::Point2d((double) prevFeaturesLeft[i].pt.x, (double) prevFeaturesLeft[i].pt.y));
//            trip_cur.push_back(cv::Point2d((double) curFeaturesLeft[i].pt.x, (double) curFeaturesLeft[i].pt.y));
//         }
//      }
//
//      printf("Inlier Points: %d\n", trip_prev.size());

      /* Extract ORB Keypoints from Current Image if number of tracked points falls below threshold */
      if(kp_prevLeft.size() < kMinFeatures){
         ExtractKeypoints_FAST(curLeft, curFeaturesLeft);
      }

      prevLeft = curLeft.clone();
      prevFeaturesLeft = curFeaturesLeft;
      count++;

      prevFinalDisplay = leftImage;

   }
}

void VisualOdometry::CalculateOdometry_ORB(ApplicationState& appState, Keyframe& kf,
                                           cv::Mat leftImage, cv::Mat rightImage, cv::Mat& cvPose)
{
   //      TriangulateStereoNormal(kp_prevLeft, kp_prevRight, prevMatchesStereo, _prevPoints3D, 0.54, 718.856);
   //      stereo->compute(curLeft, curRight, curDisparity);
   //      curDisparity.convertTo(curDisparity,CV_8U, 1.0);

   ExtractKeypoints(leftImage, orb, kp_curLeft, desc_curLeft);
   MatchKeypoints(kf.descriptor, desc_curLeft, matchesLeft);

   for (int i = matchesLeft.size() - 1; i>=0; i--)
   {
      auto m = matchesLeft[i];
      float dist = cv::norm(kp_curLeft[m.queryIdx].pt - kf.keypoints[m.trainIdx].pt);
      if (dist > 50.0f)
      {
         matchesLeft.erase(matchesLeft.begin() + i);
      }
   }

   prevPoints2D.clear();
   curPoints2D.clear();
   for(auto m : matchesLeft)
   {
      prevPoints2D.emplace_back(cv::Point2f(kf.keypoints[m.trainIdx].pt));
      curPoints2D.emplace_back(cv::Point2f(kp_curLeft[m.queryIdx].pt));
   }


   cv::Mat mask;
   cv::Mat pose = EstimateMotion_2D2D(prevPoints2D, curPoints2D, mask);
   cvPose = pose;


//   printf("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\n",
//          cvPose.at<float>(0), cvPose.at<float>(1), cvPose.at<float>(2), cvPose.at<float>(3),
//          cvPose.at<float>(4), cvPose.at<float>(5), cvPose.at<float>(6), cvPose.at<float>(7),
//          cvPose.at<float>(8), cvPose.at<float>(9), cvPose.at<float>(10), cvPose.at<float>(11));



//   cv::drawMatches(curLeft, kp_curLeft, kf.image, kf.keypoints, matchesLeft, prevFinalDisplay);

   prevLeft = curLeft.clone();
   desc_prevLeft = desc_curLeft.clone();
   kp_prevLeft = kp_curLeft;
   count++;

   prevFinalDisplay = leftImage;


}

void VisualOdometry::Update(ApplicationState& appState, Clay::Ref<Clay::TriangleMesh> axes)
{
   auto start_point = std::chrono::steady_clock::now();

   double timestamp = 0;
   if (appState.DATASET_ENABLED)
   {
      leftImage = _data->GetNextImage();
      rightImage = _data->GetNextSecondImage();
   } else
   {
      ((ImageReceiver *) this->_dataReceiver->receivers[appState.ZED_LEFT_IMAGE_RAW])->getData(leftImage, appState, timestamp);
      ((ImageReceiver *) this->_dataReceiver->receivers[appState.ZED_RIGHT_IMAGE_RAW])->getData(rightImage, appState, timestamp);
   }

   cv::Mat cvPose;

   if (!leftImage.empty() && leftImage.rows > 0 && leftImage.cols > 0 && !rightImage.empty() && rightImage.rows > 0 && rightImage.cols > 0)
   {
      if (count == 0)
      {
         width = leftImage.cols;
         height = leftImage.rows;
         cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
//         cvtColor(rightImage, prevRight, cv::COLOR_BGR2GRAY);
         ExtractKeypoints(prevLeft, orb, kp_prevLeft, desc_prevLeft);
         _keyframes.emplace_back(Keyframe(desc_prevLeft.clone(), kp_prevLeft, Eigen::Matrix4f::Identity()));
         count++;
         return;
      }

      cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
//      cvtColor(rightImage, curRight, cv::COLOR_BGR2GRAY);

      if(!_initialized)
      {
         auto kf = _keyframes[0];
         CalculateOdometry_ORB(appState, kf, curLeft, curRight, cvPose);

         Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigenPose(reinterpret_cast<float *>(cvPose.data));
         eigenPose.transposeInPlace();

         cameraPose = cameraPose * eigenPose;

         prevFinalDisplay = leftImage.clone();
         DrawMatches(prevFinalDisplay, prevPoints2D, curPoints2D);

         std::cout << eigenPose << std::endl;
         std::cout << cameraPose << std::endl;
         CLAY_LOG_INFO("Norm: {}", eigenPose.block<3,1>(0,3).norm());

         if(cameraPose.block<3,1>(0,3).norm() > 1)
         {
            _initialized = true;
            _keyframes.emplace_back(Keyframe(desc_curLeft.clone(), kp_curLeft, cameraPose));
         }
      }
      else
      {
         auto kf = _keyframes[_keyframes.size() - 1];
         CalculateOdometry_ORB(appState, kf, curLeft, curRight, cvPose);

         Eigen::Map<Eigen::Matrix<float, 4, 4>, Eigen::RowMajor> eigenPose(reinterpret_cast<float *>(cvPose.data));
         eigenPose.transposeInPlace();

         cameraPose = cameraPose * eigenPose;

         std::cout << "After Transform: " << cameraPose << std::endl;
         std::cout << "After Pose: " << eigenPose << std::endl;

         prevFinalDisplay = leftImage.clone();
         DrawMatches(prevFinalDisplay, prevPoints2D, curPoints2D);



         CLAY_LOG_INFO("Norm: {} KF: {}", eigenPose.block<3,1>(0,3).norm(), kf.keypoints.size());
         if(eigenPose.block<3,1>(0,3).norm() > 0.5)
         {
            _initialized = true;
            _keyframes.emplace_back(Keyframe(desc_curLeft.clone(), kp_curLeft, cameraPose));
            CLAY_LOG_INFO("Added New Keyframe: {}", _keyframes.size());

            glm::mat4 glmTransform;
            for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) glmTransform[j][i] = cameraPose(i, j);
            glmTransform[3][0] *= 0.03;
            glmTransform[3][1] *= 0.03;
            glmTransform[3][2] *= 0.03;
            axes->ApplyTransform(glmTransform);
         }


      }

   }





   auto end_point = std::chrono::steady_clock::now();
   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();

   float duration = (end - start) * 0.001f;

//   CLAY_LOG_INFO("(Visual Odometry) Total Time Taken: {} ms\n", duration);

//      for(int i = 0; i<_prevPoints3D.size(); i++)
//      {
//         cloud->InsertVertex(_prevPoints3D[i].GetPoint3D().x() * 0.01, _prevPoints3D[i].GetPoint3D().y() * 0.01, _prevPoints3D[i].GetPoint3D().z() * 0.01);
//      }

//      AppUtils::DisplayImage(curFinalDisplay, appState);
}

void VisualOdometry::ExtractPoseLinear()
{
   /* Calculate Essential Matrix from Correspondences and Intrinsics
          (TUM-RGBD) - fx:517.3 fy:516.5 cx:318.6 cy:255.3
          (KITTI) - fx:718.856 fy:718.856 cx:607.193 cy:185.216
          (IHMC-Chest-L515) - fx:602.259 fy:603.040 cx:321.375 cy:240.515
      */
   //   float fx = 602.259, fy = 603.040, cx = 321.375, cy = 240.515;
   //   float data[9] = {   fx, 0, cx, 0, fy, cy, 0, 0, 1 };
   //   cv::Mat K = cv::Mat(3, 3, CV_32FC1, data);
   //   cv::Mat R(3,3,CV_32FC1), t(1,3,CV_32FC1), mask;
   //
   //   cv::Mat E = findEssentialMat(kp_prevLeft, kp_curLeft, K, cv::RANSAC, 0.999, 1.0, mask);
   //
   //   /*
   //    * Recover Pose from Essential Matrix (R,t)
   //    * */
   //   recoverPose(E, kp_prevLeft, kp_curLeft, K, R, t, mask);
   //
   //   Vector3f translation;
   //   translation << t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2);

   //      ROS_DEBUG("(%.2lf, %.2lf, %.2lf)", t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2));

   //
   //      double dist = translation.norm();
   //
   //      cout << "Translation:" << t << "\tDistance:" << dist << endl;
   //      if(i > 1){
   //         /* Scale the translation */
   //      }
   //      /*
   //      * Calculate the current pose from previous pose.
   //      * */
   //      Mat cvPose = Mat::eye(4,4,CV_32FC1);
   //      R.copyTo(cvPose(Range(0,3), Range(0,3))); /* Excludes the 'end' element */
   //      t.copyTo(cvPose(Range(0,3), Range(3,4)));
   //      this->cvCurPose = (this->cvCurPose * cvPose);
   //
   //      /*
   //       * Triangulate 3D points from correspondences and Projection Matrices P = K*RT
   //       * */
   //      vector<Point2f> trip_prev, trip_cur;
   //      for(int i = 0; i < mask.rows; i++) {
   //         if(!mask.at<unsigned char>(i)){
   //            trip_prev.push_back(cv::Point2d((double)kp_prev[i].x,(double)kp_prev[i].y));
   //            trip_cur.push_back(cv::Point2d((double)kp_cur[i].x,(double)kp_cur[i].y));
   //         }
   //      }
   //
   //      printf("Inlier Points: %d\n", trip_prev.size());
   //      Mat point3d_homo;
   //      if(trip_prev.size() > 0){
   //
   //         Mat Rt0 = cvPrevPose(Range(0,3), Range(0,4));
   //         Mat Rt1 = cvCurPose(Range(0,3), Range(0,4));
   //
   //         cv::triangulatePoints(K * Rt0, K * Rt1,
   //                               trip_prev, trip_cur,
   //                               point3d_homo);
   //         printf("Triangulated Points: %d\n", point3d_homo.cols);
   //      }
   //
   //      /* Filter the 3D points for positive Z-coordinate */
   //      int count = 0;
   //      for(int m = 0; m<point3d_homo.cols; m++){
   //         Mat point = point3d_homo.col(m) / point3d_homo.at<float>(3,m);
   //         float z = point.at<float>(2);
   //         if (z > 0) {
   //            landmarks.emplace_back(Vector3f(point.at<float>(0,m), point.at<float>(1,m), point.at<float>(2,m)));
   //            count++;
   //         }
   //      }
   //      cout << "Count Z-forward: " << count << endl;
}