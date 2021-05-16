#include "FileManager.h"

void FileManager::load_sample_depth(String filename, Mat& depth)
{
   depth = imread(ros::package::getPath("map_sense") + filename, IMREAD_ANYDEPTH);
}

void FileManager::load_sample_color(String filename, Mat& color)
{
   color = imread(ros::package::getPath("map_sense") + filename, IMREAD_COLOR);
}

void FileManager::get_sample_depth(Mat depth, float mean, float stddev)
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

void FileManager::get_sample_color(Mat color)
{
   for (int i = 0; i < color.rows; i++)
   {
      for (int j = 0; j < color.cols; j++)
      {
         color.at<Vec3b>(i, j) = Vec3b(0, 0, 255);
      }
   }
}