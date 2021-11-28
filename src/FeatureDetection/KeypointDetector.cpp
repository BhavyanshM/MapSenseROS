//
// Created by quantum on 6/30/21.
//

#include "KeypointDetector.h"

KeypointDetector::KeypointDetector(int argc, char **argv, NetworkManager *network, ApplicationState& app) : _dataReceiver(network)
{
}

void KeypointDetector::extract_points(cv::Mat img, cv::Ptr<cv::ORB> orb, std::vector<cv::Point2f>& points) {
   std::vector<cv::KeyPoint> kp;
   orb->detect(img, kp, cv::noArray());
   for(int i = 0; i<kp.size(); i++){
      points.push_back(kp[i].pt);
   }
}

void KeypointDetector::track_features(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts){
   std::vector<uchar> status;
   std::vector<float> err;
   cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);

   cur_pts.clear();
   calcOpticalFlowPyrLK(prev, cur, prev_pts, cur_pts, status, err, cv::Size(21,21), 3, criteria);

   ROS_DEBUG("Previous Keypoints: (%d) \t Current Keypoints: (%d)", prev_pts.size(), cur_pts.size());

   for(uint i = 0; i < prev_pts.size(); i++)
   {
      // Select good points
      if(status[i] != 1) {
         prev_pts.erase(prev_pts.begin() + i);
         cur_pts.erase(cur_pts.begin() + i);
      }
      // else{
      //     printf("(%d)(%.2lf, %.2lf) -> (%.2lf, %.2lf)\n", i, prev_pts[i].x, prev_pts[i].y,cur_pts[i].x,cur_pts[i].y);
      // }
   }
}

void KeypointDetector::draw_matches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts){
   for (int i = 0; i<prev_pts.size(); i++){
      line(img, prev_pts[i],  cur_pts[i], cv::Scalar(0,255,0), 3);
      circle(img, prev_pts[i], 2, cv::Scalar(0,0,0), -1);
      circle(img,  cur_pts[i], 2, cv::Scalar(255, 255, 255), -1);
   }
}

void KeypointDetector::update(ApplicationState& appState)
{
   cv::Mat leftImage, rightImage;
   double timestamp = 0;

   ((ImageReceiver *) this->_dataReceiver->receivers[appState.ZED_LEFT_IMAGE_RAW])->getData(leftImage, appState, timestamp);
   ((ImageReceiver *) this->_dataReceiver->receivers[appState.ZED_RIGHT_IMAGE_RAW])->getData(rightImage, appState, timestamp);

   if(!leftImage.empty() && leftImage.rows > 0 && leftImage.cols > 0 && !rightImage.empty() && rightImage.rows > 0 && rightImage.cols > 0)
   {
      if(count == 0)
      {
         cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
         cvtColor(rightImage, prevRight, cv::COLOR_BGR2GRAY);
         extract_points(prevLeft, orb, kp_prevRight);
         extract_points(prevLeft, orb, kp_prevLeft);
         count++;
         return;
      }

      cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
      cvtColor(rightImage, curRight, cv::COLOR_BGR2GRAY);

//      extract_points(cur, orb, kp_cur);

      /* Track previously found keypoints in current frame */
      track_features(prevLeft, curLeft, kp_prevLeft, kp_curLeft);
      track_features(prevRight, curRight, kp_prevRight, kp_curRight);
      draw_matches(leftImage, kp_prevLeft, kp_curLeft);
      draw_matches(rightImage, kp_prevRight, kp_curRight);


      /* Calculate Essential Matrix from Correspondences and Intrinsics
          (TUM-RGBD) - fx:517.3 fy:516.5 cx:318.6 cy:255.3
          (KITTI) - fx:718.856 fy:718.856 cx:607.193 cy:185.216
          (IHMC-Chest-L515) - fx:602.259 fy:603.040 cx:321.375 cy:240.515
      */
      float fx = 602.259, fy = 603.040, cx = 321.375, cy = 240.515;
      float data[9] = {   fx, 0, cx, 0, fy, cy, 0, 0, 1 };
      cv::Mat K = cv::Mat(3, 3, CV_32FC1, data);
      cv::Mat R(3,3,CV_32FC1), t(1,3,CV_32FC1), mask;

      cv::Mat E = findEssentialMat(kp_prevLeft, kp_curLeft, K, cv::RANSAC, 0.999, 1.0, mask);

      /*
       * Recover Pose from Essential Matrix (R,t)
       * */
      recoverPose(E, kp_prevLeft, kp_curLeft, K, R, t, mask);

      Vector3f translation;
      translation << t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2);

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


      /* Extract ORB Keypoints from Current Image if number of tracked points falls below threshold */
      if(kp_prevLeft.size() < kMinFeatures){
         extract_points(curLeft, orb, kp_curLeft);
      }
      if(kp_prevRight.size() < kMinFeatures){
         extract_points(curRight, orb, kp_curRight);
      }

      prevLeft = curLeft.clone();
      prevRight = curRight.clone();
      kp_prevLeft = kp_curLeft;
      kp_prevRight = kp_curRight;
      count++;

      cv::Mat display;
      std::vector<cv::Mat> images;
      images.emplace_back(leftImage);
      images.emplace_back(rightImage);
      cv::hconcat(images, display);

      AppUtils::DisplayImage(display, appState);


   }



}