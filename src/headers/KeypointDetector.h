//
// Created by quantum on 6/30/21.
//

#ifndef PLOTTER3D_PY_KEYPOINTDETECTOR_H
#define PLOTTER3D_PY_KEYPOINTDETECTOR_H

#include "MapsenseHeaders.h"
#include "opencv2/opencv.hpp"
#include "NetworkManager.h"
#include "ApplicationState.h"

class KeypointDetector
{
   public:
      KeypointDetector(int argc, char **argv, NetworkManager *network, ApplicationState& app);

      void extract_points(cv::Mat img, cv::Ptr<cv::ORB> orb, std::vector<cv::Point2f>& points);
      void track_features(cv::Mat prev, cv::Mat cur, std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts);
      void draw_matches(cv::Mat& img, std::vector<cv::Point2f> prev_pts, std::vector<cv::Point2f> cur_pts);

      void update(ApplicationState& appState);

   private:
      uint32_t count = 0;
      uint32_t kFeatures = 500;
      uint32_t kMinFeatures = 300;

      cv::Ptr<cv::ORB> orb = cv::ORB::create(kFeatures);
      cv::Mat prevLeft, prevRight, curLeft, curRight;
      std::vector<cv::KeyPoint> keypoints2D;
      std::vector<cv::Point2f> kp_prevLeft, kp_prevRight, kp_curLeft, kp_curRight;

      NetworkManager* _dataReceiver;



};

#endif //PLOTTER3D_PY_KEYPOINTDETECTOR_H
