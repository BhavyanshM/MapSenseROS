//
// Created by quantum on 6/30/21.
//

#include "VisualOdometry.h"

VisualOdometry::VisualOdometry(int argc, char **argv, NetworkManager *network, ApplicationState& app, DataManager *data) : _dataReceiver(network), _data(data)
{
}

void VisualOdometry::ExtractKeypoints(cv::Mat img, cv::Ptr<cv::ORB> orb, std::vector<cv::KeyPoint>& points, cv::Mat& desc)
{
   orb->detectAndCompute(img,  cv::noArray(), points, desc);
   if(desc.type() != CV_32F)
   {
      desc.convertTo(desc, CV_32F);
   }
}

void VisualOdometry::TrackKeypoints(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts)
{
   std::vector<uchar> status;
   std::vector<float> err;
   cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);

   cur_pts.clear();
   calcOpticalFlowPyrLK(prev, cur, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3, criteria);

   ROS_DEBUG("Previous Keypoints: (%d) \t Current Keypoints: (%d)", prev_pts.size(), cur_pts.size());

   for (uint i = 0; i < prev_pts.size(); i++)
   {
      // Select good points
      if (status[i] != 1)
      {
         prev_pts.erase(prev_pts.begin() + i);
         cur_pts.erase(cur_pts.begin() + i);
      }
      // else{
      //     printf("(%d)(%.2lf, %.2lf) -> (%.2lf, %.2lf)\n", i, prev_pts[i].x, prev_pts[i].y,cur_pts[i].x,cur_pts[i].y);
      // }
   }
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

void VisualOdometry::update(ApplicationState& appState)
{
   cv::Mat leftImage, rightImage;
   double timestamp = 0;

   if (_data != nullptr)
   {
      leftImage = _data->GetNextImage();
      rightImage = _data->GetNextSecondImage();
   } else
   {
      ((ImageReceiver *) this->_dataReceiver->receivers[appState.ZED_LEFT_IMAGE_RAW])->getData(leftImage, appState, timestamp);
      ((ImageReceiver *) this->_dataReceiver->receivers[appState.ZED_RIGHT_IMAGE_RAW])->getData(rightImage, appState, timestamp);
   }

   if (!leftImage.empty() && leftImage.rows > 0 && leftImage.cols > 0 && !rightImage.empty() && rightImage.rows > 0 && rightImage.cols > 0)
   {
      if (count == 0)
      {
         cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
         cvtColor(rightImage, prevRight, cv::COLOR_BGR2GRAY);
         ExtractKeypoints(prevLeft, orb, kp_prevLeft, desc_prevLeft);
         ExtractKeypoints(prevRight, orb, kp_prevRight, desc_prevRight);
         count++;
         return;
      }

      if (count == 1)
      {
         cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
         cvtColor(rightImage, curRight, cv::COLOR_BGR2GRAY);
         ExtractKeypoints(curLeft, orb, kp_curLeft, desc_curLeft);
         ExtractKeypoints(curRight, orb, kp_curRight, desc_curRight);
         count++;
         return;
      }

      cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
      cvtColor(rightImage, curRight, cv::COLOR_BGR2GRAY);

      //      ExtractKeypoints(cur, orb, kp_cur);

      /* Track previously found keypoints in current frame */
//      TrackKeypoints(prevLeft, curLeft, kp_prevLeft, kp_curLeft);
//      TrackKeypoints(prevRight, curRight, kp_prevRight, kp_curRight);

//      cv::drawKeypoints(curLeft, kp_curLeft, curLeft);
//      cv::drawKeypoints(curRight, kp_curRight, curRight);

      MatchKeypoints(desc_prevLeft, desc_curLeft, matchesLeft);
      MatchKeypoints(desc_prevRight,desc_curRight, matchesRight);

////      -- Draw matches
//      cv::Mat img_matches;
//      cv::drawMatches( curLeft, kp_curLeft, curRight, kp_curRight, matchesRight, img_matches, cv::Scalar::all(-1),
//                   cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//      //-- Show detected matches
//      imshow("Good Matches", img_matches );
//      cv::waitKey(1);

//      DrawMatches(leftImage, kp_prevLeft, kp_curLeft);
//      DrawMatches(rightImage, kp_prevRight, kp_curRight);

      CLAY_LOG_INFO("Total Keypoints Tracked (Left): {}", kp_curLeft.size());
      CLAY_LOG_INFO("Total Keypoints Tracked (Right): {}", kp_curRight.size());

      /* Match Keypoints between Left and Right images, and include those that are optimally matched. */
      MatchKeypoints(desc_curLeft, desc_curRight, matchesStereo);

      //      /* Calculate 3D points corresponding to left-right matches found in last step. */
      //      TriangulateStereoPoints(curPoseLeft, kp_curLeft, kp_curRight, kpMatchesLeft, curPoints3D);
      //
      //      /* Solve Non-Linear Least Squares to compute motion estimate for time-pair wrt only left image. */
      //      EstimateCameraMotion(prevPoints3D, curPoints3D, kp_prevLeft, kp_curLeft, curPoseLeft);

      /*Archived*/


      /* Extract ORB Keypoints from Current Image if number of tracked points falls below threshold */
//      if (kp_prevLeft.size() < kMinFeatures)
      {
         ExtractKeypoints(curLeft, orb, kp_curLeft, desc_curLeft);
         /* Bucket Sample the Points.*/
//         GridSampleKeypoints(kp_curLeft, curLeft.cols, curLeft.rows, 40, 20);
         CLAY_LOG_INFO("Total Keypoints Found (Left): {}", kp_curLeft.size());
      }
//      if (kp_prevRight.size() < kMinFeatures)
      {
         ExtractKeypoints(curRight, orb, kp_curRight, desc_curRight);
         /* Bucket Sample the Points.*/
//         GridSampleKeypoints(kp_curRight, curRight.cols, curRight.rows, 40, 20);
         CLAY_LOG_INFO("Total Keypoints Found (Right): {}", kp_curRight.size());
      }

      prevLeft = curLeft.clone();
      prevRight = curRight.clone();
      desc_prevLeft = desc_curLeft.clone();
      desc_prevRight = desc_curRight.clone();
      kp_prevLeft = kp_curLeft;
      kp_prevRight = kp_curRight;
      count++;

      cv::Mat display;
      std::vector<cv::Mat> images;
      images.emplace_back(curLeft);
      images.emplace_back(curRight);
      cv::hconcat(images, display);

      AppUtils::DisplayImage(display, appState);
   }
}

void VisualOdometry::GridSampleKeypoints(std::vector<cv::Point2f>& keypoints, int width, int height, int xCount, int yCount)
{
   int xStep = width / xCount;
   int yStep = height / yCount;

   bool grid[yCount][xCount];
   memset(grid, false, sizeof(bool) * yCount * xCount);

   std::vector<cv::Point2f> finalKeypoints;
   for (int i = 0; i < keypoints.size(); i++)
   {
      cv::Point2f point = keypoints[i];

      if (point.x >= 0 && point.x < width && point.y >= 0 && point.y < height)
      {
         int xIndex = (int) ((float) point.x / (float) xStep);
         int yIndex = (int) ((float) point.y / (float) yStep);
         //   CLAY_LOG_INFO("i: {}, Size: {} Dims:{} {} Point: {} {} -> {} {}", i, keypoints.size(), width, height, point.x, point.y, xIndex, yIndex);

         if (xIndex >= 0 && xIndex < xCount && yIndex >= 0 && yIndex < yCount)
         {
            if (!grid[yIndex][xIndex])
            {
               finalKeypoints.push_back(keypoints[i]);
               grid[yIndex][xIndex] = true;
            }
         }
      }
   }

   keypoints.clear();
   keypoints = finalKeypoints;
}

void VisualOdometry::MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches)
{
   using namespace cv;
   Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
//   FlannBasedMatcher matcher(new flann::LshIndexParams(20, 10, 2));
//   BFMatcher matcher(NORM_L2, true);
   CLAY_LOG_INFO("Works {} {}", desc1.type(), desc2.type());
   matcher->match( desc1, desc2, matches);
   CLAY_LOG_INFO("Does'nt it?");
   //-- Filter matches using the Lowe's ratio test
//   const float ratio_thresh = 0.7f;
//   for (size_t i = 0; i < knn_matches.size(); i++)
//   {
//      if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
//      {
//         matches.push_back(knn_matches[i][0]);
//      }
//   }
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