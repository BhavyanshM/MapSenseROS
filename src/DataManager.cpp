#include "DataManager.h"
#include "AppUtils.h"


DataManager::DataManager(ApplicationState& appState, const std::string& directory, const std::string& secondDirectory, const std::string& poseFile)
    : _directory(directory), _secondDirectory(secondDirectory)
{
     AppUtils::getFileNames(_directory, _fileNames, false);
     if(secondDirectory != "") AppUtils::getFileNames(secondDirectory, _secondFileNames, false);
     if(poseFile != "")
     {
         ifstream in_file;
         in_file.open(poseFile);
         _poses = xt::load_csv<double>(in_file, ' ');

         _poses.reshape({-1, 12});

         CLAY_LOG_INFO("Poses Shape: {} {}", _poses.shape().at(0), _poses.shape().at(1));
     }
}

void DataManager::ShowNext()
{
   cv::Mat nextImage = GetNextImage();
   cv::imshow("DataManager", nextImage);
   if(_secondDirectory != "")
   {
      cv::Mat nextSecondImage = GetNextSecondImage();
      cv::imshow("DataManager Stereo", nextSecondImage);
   }
   cv::waitKey(1);
}

cv::Mat DataManager::GetNextImage()
{
//   CLAY_LOG_INFO("Loading Image: {}", _directory + _fileNames[_counter]);
   return cv::imread(_directory + _fileNames[_counter++], cv::IMREAD_COLOR);
}

cv::Mat DataManager::GetNextSecondImage()
{
//   CLAY_LOG_INFO("Loading Image: {}", _secondDirectory + _secondFileNames[_counter]);
   return cv::imread(_secondDirectory + _secondFileNames[_secondCounter++], cv::IMREAD_COLOR);
}

void DataManager::WriteScanPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan, uint32_t id)
{
   std::ofstream file;
   std::string filename = ros::package::getPath("map_sense") + "/Extras/Clouds/Scan_" + std::to_string(id);
   CLAY_LOG_INFO("Writing Regions to: {}", filename);
   file.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
   file << "DATA ascii" << std::endl;
   for (const pcl::PointXYZ& pt : scan->points)
   {
      file << boost::format("%.3f %.3f %.3f\n") % pt.x % pt.y % pt.z;
   }
   file.close();
}

cv::Mat DataManager::ReadImage(std::string filename)
{
   std::cout << "PATH:" << ros::package::getPath("map_sense") << std::endl;
   return imread(ros::package::getPath("map_sense") + filename, cv::IMREAD_COLOR);
}

void DataManager::load_sample_depth(std::string filename, cv::Mat& depth)
{
   depth = imread(ros::package::getPath("map_sense") + filename, cv::IMREAD_ANYDEPTH);
}

void DataManager::get_sample_color(cv::Mat color)
{
   for (int i = 0; i < color.rows; i++)
   {
      for (int j = 0; j < color.cols; j++)
      {
         color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
      }
   }
}

void DataManager::get_sample_depth(cv::Mat depth, float mean, float stddev)
{
   std::default_random_engine generator;
   std::normal_distribution<double> distribution(mean, stddev);
   for (int i = 0; i < depth.cols; i++)
   {
      for (int j = 0; j < depth.rows; j++)
      {
         float d = 10.04;
         d += distribution(generator);
         if (160 < i && i < 350 && 200 < j && j < 390)
         {
            // d = 0.008 * i + 0.014 * j;
            depth.at<short>(j, i) = (d - 2.0f) * 1000;
         } else
         {
            depth.at<short>(j, i) = d * 1000;
         }
      }
   }
}

void DataManager::load_sample_color(std::string filename, cv::Mat& color)
{
   color = imread(ros::package::getPath("map_sense") + filename, cv::IMREAD_COLOR);
}
