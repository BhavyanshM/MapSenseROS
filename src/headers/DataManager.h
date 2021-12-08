//
// Created by quantum on 5/15/21.
//

#ifndef FILEIO_H
#define FILEIO_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include <xtensor/xarray.hpp>
#include <xtensor/xcsv.hpp>
#include "MapsenseHeaders.h"
#include "Core/Log.h"
#include "ApplicationState.h"

class DataManager
{
   public:
      DataManager(ApplicationState& appState, const std::string& directory, const std::string& secondDirectory = "", const std::string& poseFile = "");

      void get_sample_depth(cv::Mat depth, float mean, float stddev);

      void get_sample_color(cv::Mat color);

      void load_sample_depth(std::string filename, cv::Mat& depth);

      void load_sample_color(std::string filename, cv::Mat& color);

      cv::Mat GetNextImage();

      cv::Mat GetNextSecondImage();

      void ShowNext();

      static cv::Mat ReadImage(std::string filename);

      static void WriteScanPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan, uint32_t id);

   private:
      std::string _directory, _secondDirectory;
      std::vector<std::string> _fileNames;
      std::vector<std::string> _secondFileNames;
      uint32_t _counter = 0;
      uint32_t _secondCounter = 0;
      xt::xarray<float> _poses;
};

#endif //FILEIO_H
