//
// Created by quantum on 6/30/21.
//

#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "MapsenseHeaders.h"
#include "opencv2/opencv.hpp"
#include "NetworkManager.h"
#include "ApplicationState.h"
#include "Scene/Mesh/PointCloud.h"
#include "Scene/Mesh/TriangleMesh.h"
#include "PointLandmark.h"
#include "CameraParams.h"
#include "BundleAdjustment.h"

struct Keyframe
{
   public:
      Keyframe(cv::Mat desc, std::vector<cv::KeyPoint> kp, Eigen::Matrix4f pose)
         : descLeft(desc), keypointsLeft(kp), pose(pose) {};
      Keyframe(cv::Mat descLeft, cv::Mat descRight, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, Eigen::Matrix4f pose, cv::Mat left, cv::Mat right)
            : descLeft(descLeft), descRight(descRight), keypointsLeft(kpLeft), keypointsRight(kpRight), pose(pose), leftImage(left), rightImage(right) {};
      cv::Mat descLeft, descRight;
      std::vector<cv::KeyPoint> keypointsLeft, keypointsRight;
      Eigen::Matrix4f pose;
      cv::Mat leftImage, rightImage;

};

class VisualOdometry
{
   public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VisualOdometry(int argc, char **argv, NetworkManager *network, ApplicationState& app, DataManager *data = nullptr);
      void LoadImages(ApplicationState& appState);
      void Initialize(Clay::Ref<Clay::PointCloud>& cloud);
      bool Update(ApplicationState& appState, Clay::Ref<Clay::TriangleMesh> axes, Clay::Ref<Clay::PointCloud> cloud);

      void ExtractKeypoints_FAST(cv::Mat img_1, vector<cv::Point2f>& points1);
      void ExtractKeypoints(cv::Mat img, cv::Ptr<cv::ORB> orb, std::vector<cv::KeyPoint>& points, cv::Mat& desc);
      void TrackKeypoints(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts);
      void MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches);
      void GridSampleKeypoints(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::DMatch>& matches);
      void ExtractFinalSet(std::vector<cv::DMatch> leftMatches, std::vector<cv::KeyPoint> curLeftKp, std::vector<PointLandmark>& points3D);
      void ExtractPoseLinear();

      void DrawMatches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts);
      void DrawLandmarks(cv::Mat& img, std::vector<PointLandmark>& landmarks);
      void DrawAllMatches(cv::Mat& image);
      const Eigen::Matrix4f& EstimateMotion(std::vector<PointLandmark> points, int cameraID);
      cv::Mat EstimateMotion_2D2D(std::vector<cv::Point2f>& prevFeatures, std::vector<cv::Point2f>& curFeatures, cv::Mat& mask, const CameraParams& cam);

      void Show();
      void TriangulateStereoNormal(std::vector<cv::KeyPoint>& pointsTrain, std::vector<cv::KeyPoint>& pointsQuery, std::vector<cv::DMatch>& matches,
                                   std::vector<PointLandmark>& points3D);
      void TriangulateStereoPoints(cv::Mat& leftPoseWorld, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, std::vector<cv::DMatch> stereoMatches,
                                   std::vector<PointLandmark> points3D);

      cv::Mat TriangulatePoints(std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& curPoints, const CameraParams& cam, cv::Mat relativePose);

      void CalculateOdometry_ORB(ApplicationState& appState, Keyframe& kf, cv::Mat leftImage, cv::Mat rightImage, cv::Mat& cvPose, std::vector<PointLandmark>& points3D);
      void CalculateOdometry_FAST(ApplicationState& appState, Eigen::Matrix4f& transform);
      void ImGuiUpdate(ApplicationState& app);

      cv::Mat CalculateStereoDepth(cv::Mat left, cv::Mat right);

   private:
      Eigen::Matrix4f cameraPose;

      bool _initialized = false;
      uint32_t count = 0;
      uint32_t kFeatures = 800;
      uint32_t kMinFeatures = 1000;
      uint32_t width = 0;
      uint32_t height = 0;
      uint32_t xGridCount = 60;
      uint32_t yGridCount = 30;
      float scalar = 0.03f;


      cv::Ptr<cv::ORB> orb = cv::ORB::create(kFeatures);
      cv::Mat prevLeft, prevRight, curLeft, curRight, leftImage, rightImage;
      std::vector<cv::DMatch> matchesLeft, matchesRight, prevMatchesStereo, curMatchesStereo;
      cv::Mat desc_prevRight, desc_prevLeft, desc_curRight, desc_curLeft;
      cv::Mat curFinalDisplay, prevFinalDisplay;
      cv::Mat curPoseLeft, prevPosLeft, curPoseRight, prevPoseRight;
      cv::Mat curDisparity;
      std::vector<cv::KeyPoint> kp_prevLeft, kp_prevRight, kp_curLeft, kp_curRight;
      cv::Mat cvCurPose = cv::Mat::eye(4,4, CV_32F);
      std::vector<PointLandmark> _prevPoints3D, _curPoints3D;

      cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create();
      std::vector<cv::Point2f> prevFeaturesLeft, curFeaturesLeft;
      std::vector<cv::Point2f> prevPoints2D, curPoints2D;

      BundleAdjustment* _bundleAdjustment;
      NetworkManager *_dataReceiver;
      DataManager *_data;

      std::vector<Keyframe> _keyframes;

};

//cv::Mat
//VisualOdometry::TriangulatePoints(std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& curPoints, const CameraParams& cam, cv::Mat relativePose)
//{
//   using namespace cv;
//   float data[9] = {cam._fx, 0, cam._cx, 0, cam._fy, cam._cy, 0, 0, 1};
//   Mat K = Mat(3, 3, CV_32FC1, data);
//
//   std::cout << "Pose for Triangulation:" << std::endl << relativePose << std::endl;
//
//   Mat RtCurrent = cvCurPose * relativePose;
//   Mat Rt0 = cvCurPose(Range(0, 3), Range(0, 4));
//   Mat Rt1 = RtCurrent(Range(0, 3), Range(0, 4));;
//
//   Mat points4D;
//   triangulatePoints(K * Rt0, K * Rt1, prevPoints, curPoints, points4D);
//   return points4D;
//}

//void VisualOdometry::TrackKeypoints(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts)
//{
//   std::vector<uchar> status;
//   std::vector<float> err;
//   cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);
//
//   cur_pts.clear();
//   calcOpticalFlowPyrLK(prev, cur, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3, criteria);
//
//   ROS_DEBUG("Previous Keypoints: (%d) \t Current Keypoints: (%d)", prev_pts.size(), cur_pts.size());
//
//   int lastIndex = 0;
//   for (uint i = 0; i < prev_pts.size(); i++)
//   {
//      // Select good points
//      cv::Point2f point = cur_pts.at(i - lastIndex);
//      if ((status.at(i) == 0) || (point.x < 0) || (point.y < 0))
//      {
//         prev_pts.erase(prev_pts.begin() + i);
//         cur_pts.erase(cur_pts.begin() + i);
//         lastIndex++;
//      } else if ((status.at(i) == 1))
//      {
//         float dist = norm((cur_pts[i] - prev_pts[i]));
//         if (err[i] > 3.0f || dist > 20)
//         {
//            prev_pts.erase(prev_pts.begin() + i);
//            cur_pts.erase(cur_pts.begin() + i);
//            lastIndex++;
//         }
//      }
//      // else{
//      //     printf("(%d)(%.2lf, %.2lf) -> (%.2lf, %.2lf)\n", i, prev_pts[i].x, prev_pts[i].y,cur_pts[i].x,cur_pts[i].y);
//      // }
//   }
//}

//void VisualOdometry::TriangulateStereoPoints(cv::Mat& leftPoseWorld, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight,
//                                             std::vector<cv::DMatch> stereoMatches, std::vector<PointLandmark> points3D)
//{
//   /* Calculate Essential Matrix from Correspondences and Intrinsics
//       (TUM-RGBD) - fx:517.3 fy:516.5 cx:318.6 cy:255.3
//       (KITTI) - fx:718.856 fy:718.856 cx:607.193 cy:185.216
//       (IHMC-Chest-L515) - fx:602.259 fy:603.040 cx:321.375 cy:240.515
//   */
//   using namespace cv;
//
//   std::vector<Point2f> leftPoints2D;
//   std::vector<Point2f> rightPoints2D;
//   for (auto match: stereoMatches)
//   {
//      leftPoints2D.push_back(kpLeft[match.queryIdx].pt);
//      rightPoints2D.push_back(kpRight[match.trainIdx].pt);
//   }
//
//   float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216;
//   float data[9] = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
//   cv::Mat K = cv::Mat(3, 3, CV_32FC1, data);
//   cv::Mat R(3, 3, CV_32FC1), t(1, 3, CV_32FC1), mask;
//
//   Mat Rt0 = Mat::eye(3, 4, CV_32FC1);
//   Mat Rt1 = Mat::eye(3, 4, CV_32FC1);
//   Rt1.at<float>(0, 3) = 1;
//
//   //   R.copyTo(cvPose(Range(0,3), Range(0,3))); /* Excludes the 'end' element */
//   //   t.copyTo(cvPose(Range(0,3), Range(3,4)));
//
//
//   Mat points4D;
//   cv::triangulatePoints(K * Rt0, K * Rt1, leftPoints2D, rightPoints2D, points4D);
//
//   Mat combined;
//   drawMatches(curLeft, kp_curLeft, curRight, kp_curRight, prevMatchesStereo, combined);
//   imshow("Matches", combined);
//   waitKey(1);
//
//   int count = 0;
//   for (int m = 0; m < points4D.cols; m++)
//   {
//      Mat point = points4D.col(m) / points4D.at<float>(3, m);
//      float z = point.at<float>(2);
//      //      CLAY_LOG_INFO("Triangulated Point: {} {} {}", point.at<float>(0,m), point.at<float>(1,m), point.at<float>(2,m));
//      if (z > 0 && z < 1000)
//      {
//         points3D.emplace_back(PointLandmark(Vector3f(point.at<float>(0, m), point.at<float>(1, m), point.at<float>(2, m))));
//         count++;
//      }
//   }
//   //   CLAY_LOG_INFO("After Triangulation: {} {} {} {} {}", leftPoints2D.size(), rightPoints2D.size(), points4D.rows, points4D.cols, points3D.size());
//
//}

//const Eigen::Matrix4f& VisualOdometry::EstimateMotion(std::vector<PointLandmark> points, int cameraID)
//{
//   float fx = 718.856, fy = 718.856, cx = 607.193, cy = 185.216;
//   float data[9] = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
//   cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, data);
//
//   // Read points
//   std::vector<cv::Point2f> imagePoints;
//   std::vector<cv::Point3f> objectPoints;
//
//   for (auto landmark: points)
//   {
//      imagePoints.emplace_back(cv::Point2f(landmark.GetMeasurements2D()[2].x(), landmark.GetMeasurements2D()[2].y()));
//      objectPoints.emplace_back(cv::Point3f(landmark.GetPoint3D().x(), landmark.GetPoint3D().y(), landmark.GetPoint3D().z()));
//   }
//
//   //   std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
//
//   //   std::cout << "Initial cameraMatrix: " << cameraMatrix << std::endl;
//
//   cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
//   distCoeffs.at<double>(0) = 0;
//   distCoeffs.at<double>(1) = 0;
//   distCoeffs.at<double>(2) = 0;
//   distCoeffs.at<double>(3) = 0;
//
//   cv::Mat rvec(3, 1, cv::DataType<double>::type);
//   cv::Mat tvec(3, 1, cv::DataType<double>::type);
//   cv::Mat rotation(3, 3, CV_32F);
//
//   cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false);
//   cv::Rodrigues(rvec, rotation);
//
//   cv::Mat cvPose = cv::Mat::eye(4, 4, CV_32FC1);
//   rotation.copyTo(cvPose(cv::Range(0, 3), cv::Range(0, 3))); /* Excludes the 'end' element */
//   tvec.copyTo(cvPose(cv::Range(0, 3), cv::Range(3, 4)));
//
//   cv::invert(cvPose, cvPose);
//   this->cvCurPose = (this->cvCurPose * cvPose);
//
//   //   std::cout << "Pose: " << std::endl << cvPose << std::endl;
//
//   printf("%.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf %.4lf\n", cvPose.at<float>(0), cvPose.at<float>(1), cvPose.at<float>(2),
//          cvPose.at<float>(3), cvPose.at<float>(4), cvPose.at<float>(5), cvPose.at<float>(6), cvPose.at<float>(7), cvPose.at<float>(8), cvPose.at<float>(9),
//          cvPose.at<float>(10), cvPose.at<float>(11));
//
//   return Eigen::Matrix4f::Identity();
//}
//
//void VisualOdometry::Initialize(Clay::Ref<Clay::PointCloud>& cloud)
//{
//   std::vector<PointLandmark> prevPoints3D, curPoints3D;
//
//   /* To Skip a Few Images. */
//   for (int i = 0; i < 110; i++)
//   {
//      _data->GetNextImage();
//      _data->GetNextSecondImage();
//   }
//
//   leftImage = _data->GetNextImage();
//   rightImage = _data->GetNextSecondImage();
//   width = leftImage.cols;
//   height = leftImage.rows;
//
//   auto start_point = std::chrono::steady_clock::now();
//
//   cvtColor(leftImage, prevLeft, cv::COLOR_BGR2GRAY);
//   cvtColor(rightImage, prevRight, cv::COLOR_BGR2GRAY);
//   ExtractKeypoints(prevLeft, orb, kp_prevLeft, desc_prevLeft);
//   ExtractKeypoints(prevRight, orb, kp_prevRight, desc_prevRight);
//   MatchKeypoints(desc_prevLeft, desc_prevRight, prevMatchesStereo);
//   TriangulateStereoNormal(kp_prevLeft, kp_prevRight, prevMatchesStereo, prevPoints3D);
//
//   leftImage = _data->GetNextImage();
//   rightImage = _data->GetNextSecondImage();
//   cvtColor(leftImage, curLeft, cv::COLOR_BGR2GRAY);
//   ExtractKeypoints(curLeft, orb, kp_curLeft, desc_curLeft);
//
//   MatchKeypoints(desc_prevLeft, desc_curLeft, matchesLeft);
//   ExtractFinalSet(matchesLeft, kp_curLeft, prevPoints3D);
//
//   Eigen::Matrix4f relativeCurPose = EstimateMotion(prevPoints3D, 2);
//
//   auto end_point = std::chrono::steady_clock::now();
//   long long start = std::chrono::time_point_cast<std::chrono::microseconds>(start_point).time_since_epoch().count();
//   long long end = std::chrono::time_point_cast<std::chrono::microseconds>(end_point).time_since_epoch().count();
//
//   float duration = (end - start) * 0.001f;
//
//   //   CLAY_LOG_INFO("(Visual Odometry) Total Time Taken: {} ms\n", duration);
//
//   //   cv::imshow("Initialize Visual Odometry", curFinalDisplay);
//
//   if (cloud)
//   {
//      for (int i = 0; i < prevPoints3D.size(); i++)
//      {
//         cloud->InsertVertex(prevPoints3D[i].GetPoint3D().x() * 0.01, prevPoints3D[i].GetPoint3D().y() * 0.01, prevPoints3D[i].GetPoint3D().z() * 0.01);
//      }
//   }
//
//   cv::drawMatches(prevRight, kp_prevRight, prevLeft, kp_prevLeft, prevMatchesStereo, prevFinalDisplay);
//   cv::drawMatches(curLeft, kp_curLeft, prevLeft, kp_prevLeft, matchesLeft, curFinalDisplay);
//}


















#endif //VISUAL_ODOMETRY_H
