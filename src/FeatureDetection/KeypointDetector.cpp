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

   printf("(%d,%d)\n", prev_pts.size(), cur_pts.size());
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
      line(img, prev_pts[i],  cur_pts[i], Scalar(0,255,0), 3);
      circle(img, prev_pts[i], 2, Scalar(0,0,0), -1);
      circle(img,  cur_pts[i], 2, Scalar(255, 255, 255), -1);
   }
}

void KeypointDetector::update(ApplicationState& appState)
{
   Mat img;
   double timestamp = 0;

   ImageReceiver *depthReceiver = ((ImageReceiver *) this->_dataReceiver->receivers[1]);
   depthReceiver->getData(img, appState, timestamp);

   if(!img.empty() && img.rows > 0 && img.cols > 0)
   {

      if(count == 0)
      {
         cvtColor(img, prev, cv::COLOR_BGR2GRAY);
         extract_points(prev, orb, kp_prev);
         count++;
         return;
      }


      cvtColor(img, cur, cv::COLOR_BGR2GRAY);

//      extract_points(cur, orb, kp_cur);

      /* Track previously found keypoints in current frame */
      printf("Tracking Features\n");
      track_features(prev, cur, kp_prev, kp_cur);
      draw_matches(img, kp_prev, kp_cur);
      printf("(%d,%d)\n", kp_prev.size(), kp_cur.size());

      /* Extract ORB Keypoints from Current Image if number of tracked points falls below threshold */
      if(kp_prev.size() < kMinFeatures){
         extract_points(cur, orb, kp_cur);
      }

      prev = cur.clone();
      kp_prev = kp_cur;
      count++;

      AppUtils::DisplayImage(img, appState);


   }



}