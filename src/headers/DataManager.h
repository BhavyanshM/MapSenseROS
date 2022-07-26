//
// Created by quantum on 5/15/21.
//

#ifndef FILEIO_H
#define FILEIO_H

#include <highfive/H5File.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "Log.h"
#include "ApplicationState.h"
#include "CameraParams.h"

class DataManager
{
   public:
      DataManager(ApplicationState& appState, const std::string& directory, const std::string& secondDirectory = "", const std::string& poseFile = "");

      void ReadVectorHDF5(const std::string_view name);

      void ReadBlockHDF5(const std::string_view name);

      void WriteBlockHDF5(const std::string_view name);

      void get_sample_depth(cv::Mat depth, float mean, float stddev);

      void get_sample_color(cv::Mat color);

      void load_sample_depth(std::string filename, cv::Mat& depth);

      void load_sample_color(std::string filename, cv::Mat& color);

      cv::Mat GetNextImage();

      cv::Mat GetNextSecondImage();

      void ShowNext();

      static cv::Mat ReadImage(std::string filename);

//      static void WriteScanPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan, uint32_t id);

      void SetCamera(CameraParams& leftCam, CameraParams& rightCam)
      {
          _leftCam = leftCam; _rightCam = rightCam;
          CLAY_LOG_INFO("Params: {} {} {} {}", leftCam._fx, leftCam._cx, leftCam._fy, leftCam._cy);
      };

      CameraParams& GetLeftCamera() {return _leftCam; }

      CameraParams& GetRightCamera() {return _rightCam; }

      void SetStereoBaseline(float baseline) { _baseline = baseline; }

      float GetStereoBaseline() const { return _baseline;}

   private:
      float _baseline;
      std::string _directory, _secondDirectory;
      std::vector<std::string> _fileNames;
      std::vector<std::string> _secondFileNames;
      uint32_t _counter = 600;
      uint32_t _secondCounter = 600;
      CameraParams _leftCam, _rightCam;
};

#endif //FILEIO_H
