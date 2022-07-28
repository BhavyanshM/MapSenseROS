#include "DataManager.h"
#include "AppUtils.h"
#include "filesystem"
#include "boost/filesystem.hpp"
#include "highfive/H5Easy.hpp"
#include <random>

DataManager::DataManager(ApplicationState& appState, const std::string& directory, const std::string& secondDirectory, const std::string& poseFile)
      : _directory(directory), _secondDirectory(secondDirectory), _app(appState)
{
   _leftCam.SetParams(718.856, 718.856, 607.193, 185.216);
   _rightCam.SetParams(718.856, 718.856, 607.193, 185.216);
   _baseline = 0.54;

   //    if(boost::filesystem::exists(_directory))
   //    {
   //        AppUtils::getFileNames(_directory, _fileNames, false);
   //        if(secondDirectory != "") AppUtils::getFileNames(secondDirectory, _secondFileNames, false);
   //    }

   //   WriteBlockHDF5("/tmp/new_file.h5");
}

void DataManager::LoadPointCloud(const Clay::Ref<Clay::PointCloud>& pointcloud, const std::string_view topic_name, uint16_t cloud_id)
{
   using namespace HighFive;

   File file(_app.H5_FILE_NAME, File::ReadOnly);
   Eigen::MatrixXd D2(64 * 2048, 3);
   D2 = H5Easy::load<Eigen::MatrixXd>(file, std::string(topic_name) + "/" + std::to_string(cloud_id));

   int count = 0;
   for (int i = 0; i < D2.rows(); i++)
   {
      if (D2.block<1, 3>(i, 0).norm() > 0.1f)
      {
         count++;
         std::cout << "Data Point: " << D2(i, 0) << "\t" << D2(i, 1) << "\t" << D2(i, 2) << std::endl;
         pointcloud->InsertVertex(D2(i,0), D2(i,1), D2(i,2));
      }
   }
   std::cout << "Total Points Loaded: " << count << std::endl;
}

void DataManager::ReadBlockHDF5(const std::string_view name)
{
   using namespace HighFive;

   File file(std::string(name), File::ReadOnly);
   Eigen::MatrixXd D2(64 * 2048, 3);
   D2 = H5Easy::load<Eigen::MatrixXd>(file, "/os_cloud_node/points/0");

   int count = 0;
   for (int i = 0; i < D2.rows(); i++)
   {
      if (D2.block<1, 3>(i, 0).norm() > 0.1f)
      {
         count++;
         std::cout << "Dataset Datatype: " << D2(i, 0) << "\t" << D2(i, 1) << "\t" << D2(i, 2) << std::endl;
      }
   }
   std::cout << "Total Points: " << count << std::endl;
}

void DataManager::ReadVectorHDF5(const std::string_view name)
{
   using namespace HighFive;
   File file(std::string(name), File::ReadOnly);

   DataSet dataset = file.getDataSet("/os_cloud_node/points/0");
   std::vector<std::vector<float>> points;
   dataset.read(points);
}

void DataManager::WriteBlockHDF5(const std::string_view name)
{
   using namespace HighFive;
   // we create a new hdf5 file
   File file(std::string(name), File::ReadWrite | File::Create | File::Truncate);

   std::vector<int> data(50, 1);

   data[0] = 0;
   data[1] = 1;
   for (int i = 2; i < 50; i++)
   {
      data[i] = data[i - 1] + data[i - 2];
   }

   // let's create a dataset of native integer with the size of the vector 'data'
   DataSet dataset = file.createDataSet<int>("/dataset_one", DataSpace::From(data));

   // let's write our vector of int to the HDF5 dataset
   dataset.write(data);
}

void DataManager::ShowNext()
{
   cv::Mat nextImage = GetNextImage();
   cv::imshow("DataManager", nextImage);
   if (_secondDirectory != "")
   {
      cv::Mat nextSecondImage = GetNextSecondImage();
      cv::imshow("DataManager Stereo", nextSecondImage);
   }
   cv::waitKey(1);
}

cv::Mat DataManager::GetNextImage()
{
   //   MS_INFO("Loading Image: {}", _directory + _fileNames[_counter]);
   return cv::imread(_directory + _fileNames[_counter++], cv::IMREAD_COLOR);
}

cv::Mat DataManager::GetNextSecondImage()
{
   //   MS_INFO("Loading Image: {}", _secondDirectory + _secondFileNames[_counter]);
   return cv::imread(_secondDirectory + _secondFileNames[_secondCounter++], cv::IMREAD_COLOR);
}

//void DataManager::WriteScanPoints(pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan, uint32_t id)
//{
//   std::ofstream file;
//   std::string filename = ros::package::getPath("map_sense") + "/Extras/Clouds/Scan_" + std::to_string(id);
//   MS_INFO("Writing Regions to: {}", filename);
//   file.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
//   file << "DATA ascii" << std::endl;
//   for (const pcl::PointXYZ& pt : scan->points)
//   {
//      file << boost::format("%.3f %.3f %.3f\n") % pt.x % pt.y % pt.z;
//   }
//   file.close();
//}

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
