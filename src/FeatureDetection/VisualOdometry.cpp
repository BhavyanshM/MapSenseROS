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
//   if(desc.type() != CV_32F)
//   {
//      desc.convertTo(desc, CV_32F);
//   }
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
//   for (int i = 0; i < matchesStereo.size(); i++)
//   {
//      cv::line(combined, kp_curLeft[matchesStereo[i].queryIdx].pt, kp_curRight[matchesStereo[i].trainIdx].pt + dims, cv::Scalar(255, 150, 0), 1);
////      cv::circle(combined, prev_pts[i], 2, cv::Scalar(0, 0, 0), -1);
////      cv::circle(combined, cur_pts[i], 2, cv::Scalar(255, 255, 255), -1);
//   }
   image = combined;
}

void VisualOdometry::MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches)
{
   using namespace cv;
   BFMatcher matcher(NORM_HAMMING, true);
   matcher.match( desc1, desc2, matches);

   std::sort(matches.begin(), matches.end(), [&](const cv::DMatch& a, const cv::DMatch& b)
   {
      return a.distance < b.distance;
   });

   if(matches.size() > 150) matches.resize(150);

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
                             std::vector<cv::Point3f> points3D)
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
   drawMatches(curLeft, kp_curLeft, curRight, kp_curRight, matchesStereo, combined);
   imshow("Matches", combined);
   waitKey(1);

   int count = 0;
   for(int m = 0; m<points4D.cols; m++){
      Mat point = points4D.col(m) / points4D.at<float>(3,m);
      float z = point.at<float>(2);
//      CLAY_LOG_INFO("Triangulated Point: {} {} {}", point.at<float>(0,m), point.at<float>(1,m), point.at<float>(2,m));
      if (z > 0) {
         points3D.emplace_back(Point3f(point.at<float>(0,m), point.at<float>(1,m), point.at<float>(2,m)));
         count++;
      }
   }
   CLAY_LOG_INFO("After Triangulation: {} {} {} {} {}", leftPoints2D.size(), rightPoints2D.size(), points4D.rows, points4D.cols, points3D.size());

}

void VisualOdometry::update(ApplicationState& appState)
{
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

   if (!leftImage.empty() && leftImage.rows > 0 && leftImage.cols > 0 && !rightImage.empty() && rightImage.rows > 0 && rightImage.cols > 0)
   {
      if (count == 0)
      {
         width = leftImage.cols;
         height = leftImage.rows;
         cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
         cvtColor(rightImage, prevRight, cv::COLOR_BGR2GRAY);
         ExtractKeypoints(prevLeft, orb, kp_prevLeft, desc_prevLeft);
         ExtractKeypoints(prevRight, orb, kp_prevRight, desc_prevRight);
         count++;
         return;
      }

      cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
      cvtColor(rightImage, curRight, cv::COLOR_BGR2GRAY);
      ExtractKeypoints(curLeft, orb, kp_curLeft, desc_curLeft);
      ExtractKeypoints(curRight, orb, kp_curRight, desc_curRight);

      /* Match previously found keypoints in current frame */
      MatchKeypoints(desc_prevLeft, desc_curLeft, matchesLeft);
      MatchKeypoints(desc_prevRight,desc_curRight, matchesRight);

//      cv::drawMatches(prevLeft, kp_prevLeft, curLeft, kp_curLeft, matchesLeft, finalDisplay);

      CLAY_LOG_INFO("Total Matches (Left): {}", matchesLeft.size());
      CLAY_LOG_INFO("Total Matches (Right): {}", matchesRight.size());

//      GridSampleKeypoints(kp_curLeft, matchesLeft);
//      GridSampleKeypoints(kp_curRight, matchesRight);

      CLAY_LOG_INFO("Sampled Matches (Left): {}", matchesLeft.size());
      CLAY_LOG_INFO("Sampled Matches (Right): {}", matchesRight.size());

      /* Match Keypoints between Left and Right images, and include those that are optimally matched. */
      MatchKeypoints(desc_curLeft, desc_curRight, matchesStereo);


      DrawAllMatches(finalDisplay);


      /* Calculate 3D points corresponding to left-right matches found in last step. */
      std::vector<cv::Point3f> points3D;
      TriangulateStereoPoints(curPoseLeft, kp_curLeft, kp_curRight, matchesStereo, points3D);

      CLAY_LOG_INFO("Triangulated Points: {}", points3D.size());

      //      /* Solve Non-Linear Least Squares to compute motion estimate for time-pair wrt only left image. */
      //      EstimateCameraMotion(prevPoints3D, curPoints3D, kp_prevLeft, kp_curLeft, curPoseLeft);

      /*Archived*/


//      /* Extract ORB Keypoints from Current Image if number of tracked points falls below threshold */
//      if (matchesLeft.size() < kMinFeatures)
//      {
//
//         /* Bucket Sample the Points.*/
//
//         CLAY_LOG_INFO("Total Keypoints Found (Left): {}", kp_curLeft.size());
//      }
//      if (matchesRight.size() < kMinFeatures)
//      {
//         /* Bucket Sample the Points.*/
//
//         CLAY_LOG_INFO("Total Keypoints Found (Right): {}", kp_curRight.size());
//      }

      prevLeft = curLeft.clone();
      prevRight = curRight.clone();
      desc_prevLeft = desc_curLeft.clone();
      desc_prevRight = desc_curRight.clone();
      kp_prevLeft = kp_curLeft;
      kp_prevRight = kp_curRight;
      count++;

      AppUtils::DisplayImage(finalDisplay, appState);
   }
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