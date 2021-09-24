//
// Created by quantum on 5/15/21.
//

#ifndef FILEIO_H
#define FILEIO_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "MapsenseHeaders.h"

class FileManager
{
   public:
      void get_sample_depth(cv::Mat depth, float mean, float stddev);

      void get_sample_color(cv::Mat color);

      void load_sample_depth(std::string filename, cv::Mat& depth);

      void load_sample_color(std::string filename, cv::Mat& color);

      static cv::Mat ReadImage(std::string filename);
};

#endif //FILEIO_H
