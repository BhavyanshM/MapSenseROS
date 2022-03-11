//
// Created by quantum on 2/16/21.
//

#include "AppUtils.h"

void AppUtils::getFileNames(std::string dirName, std::vector<std::string>& files, bool printList)
{
   if (auto dir = opendir(dirName.c_str()))
   {
      while (auto f = readdir(dir))
      {

         if (!f->d_name || f->d_name[0] == '.')
            continue;
         files.emplace_back(f->d_name);
      }
      closedir(dir);
   }
   sort(files.begin(), files.end());

   if (printList)
   {
      printf("[");
      for (std::string file : files)
      {
         printf("%s, ", file.c_str());
      }
      printf("]\n");
   }
}

void AppUtils::capture_data(std::string projectPath, std::string filename, cv::Mat depth, cv::Mat color, cv::Mat filteredDepth, cv::Mat components, ApplicationState appState,
                            std::vector<std::shared_ptr<PlanarRegion>> regions)
{
   cv::Mat finalDepth, finalFilteredDepth;
   depth.convertTo(finalDepth, -1, appState.DEPTH_BRIGHTNESS, appState.DEPTH_DISPLAY_OFFSET);
   filteredDepth.convertTo(finalFilteredDepth, -1, appState.DEPTH_BRIGHTNESS, appState.DEPTH_DISPLAY_OFFSET);
   imwrite(projectPath + filename + "_Depth.png", finalDepth);
   imwrite(projectPath + filename + "_Color.png", color);
   imwrite(projectPath + filename + "_FilteredDepth.png", finalFilteredDepth);
   imwrite(projectPath + filename + "_Components.png", components);
   GeomTools::SaveRegions(regions, projectPath + "/Extras/Regions/" + std::string(4 - std::to_string(0).length(), '0').append(std::to_string(0)) + ".txt");
}

void AppUtils::appendToDebugOutput(cv::Mat disp)
{
   if (disp.rows <= 0 || disp.cols <= 0)
   {
      ROS_WARN("Image to be displayed has size 0!");
      return;
   }
   if (disp.type() == CV_8UC1)
      cvtColor(disp, disp, cv::COLOR_GRAY2BGR);
   if (disp.type() == CV_16UC3 || disp.type() == CV_16UC1)
   {
      disp.convertTo(disp, CV_8U, 0.00390625);
      disp.convertTo(disp, CV_8UC3);
   }
   cv::resize(disp, disp, cv::Size(640 * 480 / disp.rows, 480));
   images.emplace_back(disp);
   ROS_DEBUG("Appending To Debug Display: Type: %d, Rows: %d, Cols: %d, Images: %d\n", disp.type(), disp.rows, disp.cols, images.size());
}

void AppUtils::displayDebugOutput(ApplicationState appState)
{
   ROS_DEBUG("Displaying Debug Output: %d", images.size());
   hconcat(images, debugOutput);
   if (debugOutput.cols > 0 && debugOutput.rows > 0 && !debugOutput.empty())
   {
      cv::namedWindow("DebugOutput", cv::WINDOW_NORMAL);
      cv::resizeWindow("DebugOutput", (int) (debugOutput.cols * appState.DISPLAY_WINDOW_SIZE), (int) (debugOutput.rows * appState.DISPLAY_WINDOW_SIZE));
      cv::imshow("DebugOutput", debugOutput);
      cv::waitKey(1);
   }
   clearDebug();
}

void AppUtils::clearDebug()
{
   images.clear();
}

void AppUtils::setDisplayResolution(uint16_t rows, uint16_t cols)
{
   displayOutput = cv::Mat(rows, cols, CV_8UC3);
   displayOutput.setTo(0);
}

void AppUtils::canvasToMat(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos, uint8_t windowSize)
{
   for (int i = 0; i < canvas.rows(); i++)
   {
      for (int j = 0; j < canvas.cols(); j++)
      {
         if (canvas(i, j) == 1)
            circle(AppUtils::displayOutput, cv::Point(i * 6, j * 6), 2, cv::Scalar(255, 0, 0), -1);
         else if (windowPos.x() != -1 && windowPos.y() != -1 && i > windowPos.x() - windowSize && i < windowPos.x() + windowSize &&
                  j > windowPos.y() - windowSize && j < windowPos.y() + windowSize)
         {
            circle(AppUtils::displayOutput, cv::Point(i * 6, j * 6), 2, cv::Scalar(0, 0, 255), -1);
         }
      }
   }
}

void AppUtils::displayPointSet2D(std::vector<Eigen::Vector2f> points, Eigen::Vector2f offset, int scale)
{
   displayOutput.setTo(0);
   for (int i = 0; i < points.size(); i++)
   {
      circle(AppUtils::displayOutput, cv::Point((int) (points[i].x() * scale + offset.x()) * 6, (int) (points[i].y() * scale + offset.y()) * 6), 2,
             cv::Scalar(255, 0, 0), -1);
      display(20);
   }
}

void AppUtils::displayCanvasWithWindow(BoolDynamicMatrix canvas, Eigen::Vector2i windowPos, uint8_t windowSize)
{
   canvasToMat(canvas, windowPos, windowSize);
   display(20);
}

void AppUtils::display(uint16_t delay)
{
   if (displayOutput.cols > 0 && displayOutput.rows > 0 && !displayOutput.empty())
   {
      cv::namedWindow("Display", cv::WINDOW_NORMAL);
      cv::resizeWindow("Display", (int) (displayOutput.cols), (int) (displayOutput.rows));
      cv::imshow("Display", displayOutput);
      cv::waitKey(delay);
   }
}

void AppUtils::checkMemoryLimits()
{
   struct rlimit old_lim, lim, new_lim;

   // Get old limits
   if (getrlimit(RLIMIT_NOFILE, &old_lim) == 0)
   {
      printf("Old limits -> soft limit= %ld \t"
             " hard limit= %ld \n", old_lim.rlim_cur, old_lim.rlim_max);
   } else
   {
      fprintf(stderr, "%s\n", strerror(errno));
   }

   // Set new value
   lim.rlim_cur = 1024 * 1024 * 1024;
   lim.rlim_max = 1024 * 1024 * 1024;

   // Set limits
   if (setrlimit(RLIMIT_NOFILE, &lim) == -1)
   {
      fprintf(stderr, "%s\n", strerror(errno));
   }
   // Get new limits
   if (getrlimit(RLIMIT_NOFILE, &new_lim) == 0)
   {
      printf("New limits -> soft limit= %ld "
             "\t hard limit= %ld \n", new_lim.rlim_cur, new_lim.rlim_max);
   } else
   {
      fprintf(stderr, "%s\n", strerror(errno));
   }
}

void AppUtils::DisplayImage(cv::Mat disp, const ApplicationState& app, const std::string& title)
{
   if (disp.cols > 0 && disp.rows > 0 && !disp.empty())
   {
      cv::namedWindow(title, cv::WINDOW_NORMAL);
      cv::resizeWindow(title, (int) (disp.cols * app.DISPLAY_WINDOW_SIZE), (int) (disp.rows * app.DISPLAY_WINDOW_SIZE));
      cv::imshow(title, disp);
      cv::waitKey(1);
   }
}

void AppUtils::PrintMatR8(cv::Mat& mat, int value, bool invert, bool constant, int rowLimit, int colLimit)
{
   int rows = rowLimit;
   int cols = colLimit;
   if (rowLimit == 0)
      rows = mat.rows;
   if (colLimit == 0)
      cols = mat.cols;

   for (int i = 0; i < rows; i++)
   {
      for (int j = 0; j < cols; j++)
      {
         uint8_t current = mat.at<uint8_t>(i, j);

         if(constant)
         {
             if(current > (uint8_t)value)
                 printf("1 ");
             else
                 printf("0 ");
         }
         else
         {
             printf("%hhu ", current);
         }

      }
      printf("\n", i);
   }
}

void AppUtils::PrintMatR16(cv::Mat& mat, int value, bool invert, int rowLimit, int colLimit, bool linear)
{
   int rows = rowLimit;
   int cols = colLimit;
   if (rowLimit == 0)
      rows = mat.rows;
   if (colLimit == 0)
      cols = mat.cols;

   if(!linear)
   {
      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < cols; j++)
         {
            uint16_t current = mat.at<uint16_t>(i, j);
            printf("%hu ", current);
         }
         printf("\n");
      }
   } else
   {
      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < cols; j++)
         {
            uint16_t current = mat.at<uint16_t>(i, j);
            printf("(%d %d): %.2lf\n", i, j, current);
         }
         printf("\n");
      }
   }
}

void AppUtils::CalculateAndPrintStatsMat(cv::Mat& mat)
{
   int max = 0;
   int average = 0;
   int counts[] = {0,0,0,0,0,0,0,0};
   for(int i = 0; i<mat.rows; i++)
   {
      for(int j = 0; j<mat.cols; j++)
      {
         if (mat.at<char>(i,j) > max)
         {
            max = mat.at<char>(i,j);
         }
         counts[mat.at<char>(i,j)] += 1;
         average += mat.at<char>(i,j);
      }
   }

   printf("MAXIMUM: %d, AVERAGE: %.2lf\n", max, (float)average / (float)(mat.rows * mat.cols));

   for(int i = 0; i<8; i++)
   {
      printf("COUNT: %d\n", counts[i]);
   }
}