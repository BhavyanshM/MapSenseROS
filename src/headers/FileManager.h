//
// Created by quantum on 5/15/21.
//

#ifndef FILEIO_H
#define FILEIO_H

#include "ros/ros.h"
#include "ros/package.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <sstream>
#include <random>

using namespace cv;

class FileManager
{
   public:
      void get_sample_depth(Mat depth, float mean, float stddev);

      void get_sample_color(Mat color);

      void load_sample_depth(String filename, Mat& depth);

      void load_sample_color(String filename, Mat& color);
};

#endif //FILEIO_H
