//
// Created by quantum on 6/30/21.
//

#ifndef PLOTTER3D_PY_KEYPOINTDETECTOR_H
#define PLOTTER3D_PY_KEYPOINTDETECTOR_H

#include "MapsenseHeaders.h"
#include "opencv2/opencv.hpp"
#include "NetworkManager.h"
#include "ApplicationState.h"
#include "Scene/Mesh/PointCloud.h"
#include "Scene/Mesh/TriangleMesh.h"
#include "PointLandmark.h"
#include "CameraParams.h"

struct Keyframe
{
   public:
      Keyframe(cv::Mat desc, std::vector<cv::KeyPoint> kp, Eigen::Matrix4f pose)
         : descriptor(desc), keypoints(kp), pose(pose) {};
      cv::Mat descriptor;
      std::vector<cv::KeyPoint> keypoints;
      Eigen::Matrix4f pose;
      cv::Mat image;

};

class VisualOdometry
{
   public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      VisualOdometry(int argc, char **argv, NetworkManager *network, ApplicationState& app, DataManager *data = nullptr);
      void Initialize(Clay::Ref<Clay::PointCloud>& cloud);
      void Update(ApplicationState& appState, Clay::Ref<Clay::TriangleMesh> axes, Clay::Ref<Clay::PointCloud> cloud);

      void ExtractKeypoints_FAST(cv::Mat img_1, vector<cv::Point2f>& points1);
      void ExtractKeypoints(cv::Mat img, cv::Ptr<cv::ORB> orb, std::vector<cv::KeyPoint>& points, cv::Mat& desc);
      void TrackKeypoints(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts);
      void MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches);
      void GridSampleKeypoints(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::DMatch>& matches);
      void ExtractFinalSet(std::vector<cv::DMatch> leftMatches, std::vector<cv::KeyPoint> curLeftKp, std::vector<PointLandmark>& points3D);
      void ExtractPoseLinear();

      void DrawMatches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts);
      void DrawAllMatches(cv::Mat& image);
      const Eigen::Matrix4f& EstimateMotion(std::vector<PointLandmark> points, int cameraID);
      cv::Mat EstimateMotion_2D2D(std::vector<cv::Point2f>& prevFeatures, std::vector<cv::Point2f>& curFeatures, cv::Mat& mask, const CameraParams& cam);

      void Show();
      void TriangulateStereoNormal(std::vector<cv::KeyPoint>& pointsTrain, std::vector<cv::KeyPoint>& pointsQuery, std::vector<cv::DMatch>& matches,
                                   std::vector<PointLandmark>& points3D, float baseline, float focalLength);
      void TriangulateStereoPoints(cv::Mat& leftPoseWorld, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, std::vector<cv::DMatch> stereoMatches,
                                   std::vector<PointLandmark> points3D);

      cv::Mat TriangulatePoints(std::vector<cv::Point2f>& prevPoints, std::vector<cv::Point2f>& curPoints, const CameraParams& cam, cv::Mat relativePose);

      void CalculateOdometry_ORB(ApplicationState& appState, Keyframe& kf, cv::Mat leftImage, cv::Mat rightImage, cv::Mat& cvPose, cv::Mat& points4D);
      void CalculateOdometry_FAST(ApplicationState& appState, Eigen::Matrix4f& transform);
      void ImGuiUpdate(ApplicationState& app);

      cv::Mat CalculateStereoDepth(cv::Mat left, cv::Mat right);

   private:
      Eigen::Matrix4f cameraPose;

      bool _initialized = false;
      uint32_t count = 0;
      uint32_t kFeatures = 2400;
      uint32_t kMinFeatures = 100;
      uint32_t width = 0;
      uint32_t height = 0;
      uint32_t xGridCount = 60;
      uint32_t yGridCount = 30;


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

      NetworkManager *_dataReceiver;
      DataManager *_data;

      std::vector<Keyframe> _keyframes;



};

#endif //PLOTTER3D_PY_KEYPOINTDETECTOR_H
