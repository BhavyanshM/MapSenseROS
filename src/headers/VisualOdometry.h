//
// Created by quantum on 6/30/21.
//

#ifndef PLOTTER3D_PY_KEYPOINTDETECTOR_H
#define PLOTTER3D_PY_KEYPOINTDETECTOR_H

#include "MapsenseHeaders.h"
#include "opencv2/opencv.hpp"
#include "NetworkManager.h"
#include "ApplicationState.h"

class VisualOdometry
{
   public:
      VisualOdometry(int argc, char **argv, NetworkManager *network, ApplicationState& app, DataManager *data = nullptr);
      void ExtractKeypoints(cv::Mat img, cv::Ptr<cv::ORB> orb, std::vector<cv::KeyPoint>& points, cv::Mat& desc);
      void TrackKeypoints(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts);
      void DrawMatches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts);
      void update(ApplicationState& appState);
      void ExtractPoseLinear();
      void GridSampleKeypoints(std::vector<cv::Point2f>& keypoints, int width, int height, int xCount, int yCount);
      void MatchKeypoints(cv::Mat& desc1, cv::Mat& desc2, std::vector<cv::DMatch>& matches);

   private:
      uint32_t count = 0;
      uint32_t kFeatures = 300;
      uint32_t kMinFeatures = 100;

      cv::Ptr<cv::ORB> orb = cv::ORB::create(kFeatures);
      cv::Mat prevLeft, prevRight, curLeft, curRight;
      std::vector<cv::KeyPoint> keypoints2D;
      std::vector<cv::DMatch> matchesLeft, matchesRight, matchesStereo;
      cv::Mat desc_prevRight, desc_prevLeft, desc_curRight, desc_curLeft;
      std::vector<cv::KeyPoint> kp_prevLeft, kp_prevRight, kp_curLeft, kp_curRight;

      NetworkManager *_dataReceiver;
      DataManager *_data;
};

#endif //PLOTTER3D_PY_KEYPOINTDETECTOR_H
