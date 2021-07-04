//
// Created by quantum on 2/16/21.
//

#include "AppUtils.h"

void AppUtils::getFileNames(string dirName, vector<string>& files, bool printList)
{
   /* Get the sorted list of file names for planar regions at different frames. */
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
      for (string file : files)
      {
         printf("%s, ", file.c_str());
      }
      printf("]\n");
   }
}

void AppUtils::capture_data(String projectPath, String filename, Mat depth, Mat color, Mat filteredDepth, Mat components, ApplicationState appState,
                            vector<shared_ptr<PlanarRegion>> regions)
{
   Mat finalDepth, finalFilteredDepth;
   depth.convertTo(finalDepth, -1, appState.DEPTH_BRIGHTNESS, appState.DEPTH_DISPLAY_OFFSET);
   filteredDepth.convertTo(finalFilteredDepth, -1, appState.DEPTH_BRIGHTNESS, appState.DEPTH_DISPLAY_OFFSET);
   imwrite(projectPath + filename + "_Depth.png", finalDepth);
   imwrite(projectPath + filename + "_Color.png", color);
   imwrite(projectPath + filename + "_FilteredDepth.png", finalFilteredDepth);
   imwrite(projectPath + filename + "_Components.png", components);
   GeomTools::saveRegions(regions, projectPath + "/Extras/Regions/" + string(4 - to_string(0).length(), '0').append(to_string(0)) + ".txt");
}

void AppUtils::appendToDebugOutput(Mat disp)
{
   if (disp.rows <= 0 || disp.cols <= 0)
   {
      ROS_WARN("Image to be displayed has size 0!");
      return;
   }
   if (disp.type() == CV_8UC1)
      cvtColor(disp, disp, COLOR_GRAY2BGR);
   if (disp.type() == CV_16UC3 || disp.type() == CV_16UC1)
   {
      disp.convertTo(disp, CV_8U, 0.00390625);
      disp.convertTo(disp, CV_8UC3);
   }
   cv::resize(disp, disp, Size(640 * 480 / disp.rows, 480));
   images.emplace_back(disp);
   ROS_DEBUG("Appending To Debug Display: Type: %d, Rows: %d, Cols: %d, Images: %d\n", disp.type(), disp.rows, disp.cols, images.size());
}

void AppUtils::displayDebugOutput(ApplicationState appState)
{
   ROS_DEBUG("Displaying Debug Output: %d", images.size());
   hconcat(images, debugOutput);
   if (debugOutput.cols > 0 && debugOutput.rows > 0 && !debugOutput.empty())
   {
      namedWindow("DebugOutput", WINDOW_NORMAL);
      resizeWindow("DebugOutput", (int) (debugOutput.cols * appState.DISPLAY_WINDOW_SIZE), (int) (debugOutput.rows * appState.DISPLAY_WINDOW_SIZE));
      imshow("DebugOutput", debugOutput);
      waitKey(1);
   }
   clearDebug();
}

void AppUtils::clearDebug()
{
   images.clear();
}

void AppUtils::setDisplayResolution(uint16_t rows, uint16_t cols)
{
   displayOutput = Mat(rows, cols, CV_8UC3);
   displayOutput.setTo(0);
}

void AppUtils::canvasToMat(BoolDynamicMatrix canvas, Vector2i windowPos, uint8_t windowSize)
{
   for (int i = 0; i < canvas.rows(); i++)
   {
      for (int j = 0; j < canvas.cols(); j++)
      {
         if (canvas(i, j) == 1)
            circle(AppUtils::displayOutput, Point(i * 6, j * 6), 2, Scalar(255, 0, 0), -1);
         else if (windowPos.x() != -1 && windowPos.y() != -1 && i > windowPos.x() - windowSize && i < windowPos.x() + windowSize &&
                  j > windowPos.y() - windowSize && j < windowPos.y() + windowSize)
         {
            circle(AppUtils::displayOutput, Point(i * 6, j * 6), 2, Scalar(0, 0, 255), -1);
         }
      }
   }
}

void AppUtils::displayPointSet2D(vector<Vector2f> points, Vector2f offset, int scale)
{
   displayOutput.setTo(0);
   for (int i = 0; i < points.size(); i++)
   {
      circle(AppUtils::displayOutput, Point((int) (points[i].x() * scale + offset.x()) * 6, (int) (points[i].y() * scale + offset.y()) * 6), 2,
             Scalar(255, 0, 0), -1);
      display(20);
   }
}

void AppUtils::displayCanvasWithWindow(BoolDynamicMatrix canvas, Vector2i windowPos, uint8_t windowSize)
{
   canvasToMat(canvas, windowPos, windowSize);
   display(20);
}

void AppUtils::display(uint16_t delay)
{
   if (displayOutput.cols > 0 && displayOutput.rows > 0 && !displayOutput.empty())
   {
      namedWindow("Display", WINDOW_NORMAL);
      resizeWindow("Display", (int) (displayOutput.cols), (int) (displayOutput.rows));
      imshow("Display", displayOutput);
      waitKey(delay);
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

void AppUtils::DisplayImage(Mat disp, const ApplicationState& app)
{
   if (disp.cols > 0 && disp.rows > 0 && !disp.empty())
   {
      namedWindow("DisplayImage", WINDOW_NORMAL);
      resizeWindow("DisplayImage", (int) (disp.cols * app.DISPLAY_WINDOW_SIZE), (int) (disp.rows * app.DISPLAY_WINDOW_SIZE));
      imshow("DisplayImage", disp);
      waitKey(1);
   }
}

