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

class VisualOdometry
{
   public:
      VisualOdometry(int argc, char **argv, NetworkManager *network, ApplicationState& app, DataManager *data = nullptr);
      void Initialize(Clay::Ref<Clay::PointCloud>& cloud);
      void update(ApplicationState& appState);

      void ExtractKeypoints(cv::Mat img, cv::Ptr<cv::ORB> orb, std::vector<cv::KeyPoint>& points, cv::Mat& desc);
      void TrackKeypoints(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts);
      void MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches);
      void GridSampleKeypoints(std::vector<cv::KeyPoint>& keypoints, std::vector<cv::DMatch>& matches);
      void ExtractPoseLinear();
      void DrawMatches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts);
      void DrawAllMatches(cv::Mat& image);
      void Show();

      void TriangulateStereoNormal(std::vector<cv::KeyPoint>& pointsTrain, std::vector<cv::KeyPoint>& pointsQuery, std::vector<cv::DMatch>& matches,
                                   std::vector<cv::Point3f>& points3D, float baseline, float focalLength);
      void TriangulateStereoPoints(cv::Mat& leftPoseWorld, std::vector<cv::KeyPoint> kpLeft, std::vector<cv::KeyPoint> kpRight, std::vector<cv::DMatch> stereoMatches,
                                   std::vector<cv::Point3f> points3D);

   private:
      uint32_t count = 0;
      uint32_t kFeatures = 400;
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
      std::vector<cv::KeyPoint> kp_prevLeft, kp_prevRight, kp_curLeft, kp_curRight;

      NetworkManager *_dataReceiver;
      DataManager *_data;
};

#endif //PLOTTER3D_PY_KEYPOINTDETECTOR_H
