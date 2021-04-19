//
// Created by quantum on 2/16/21.
//

#include "AppUtils.h"

void AppUtils::write_regions(vector<shared_ptr<PlanarRegion>> regions, int frameId)
{
   ofstream file;
   string filename = ros::package::getPath("map_sense") + "/Extras/Regions/" + string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt";
   file.open(filename, fstream::in | fstream::out | fstream::app);
   file << "NumRegions:" << regions.size() << endl;
   for (shared_ptr<PlanarRegion> region : regions)
   {
      region->writeToFile(file);
   }
   file.close();
   cout << "Writing Regions to:"
        << ros::package::getPath("map_sense") + "/Extras/Regions/" + string(4 - to_string(frameId).length(), '0').append(to_string(frameId)) + ".txt" << endl;
}

void AppUtils::capture_data(String filename, Mat depth, Mat color, Mat filteredDepth, Mat components, ApplicationState appState,
                            vector<shared_ptr<PlanarRegion>> regions)
{
   Mat finalDepth, finalFilteredDepth;
   depth.convertTo(finalDepth, -1, appState.DEPTH_BRIGHTNESS, appState.DEPTH_DISPLAY_OFFSET);
   filteredDepth.convertTo(finalFilteredDepth, -1, appState.DEPTH_BRIGHTNESS, appState.DEPTH_DISPLAY_OFFSET);
   imwrite(ros::package::getPath("map_sense") + filename + "_Depth.png", finalDepth);
   imwrite(ros::package::getPath("map_sense") + filename + "_Color.png", color);
   imwrite(ros::package::getPath("map_sense") + filename + "_FilteredDepth.png", finalFilteredDepth);
   imwrite(ros::package::getPath("map_sense") + filename + "_Components.png", components);
   write_regions(regions, 0);
}

void AppUtils::appendToDebugOutput(Mat disp)
{
   if(disp.type() == CV_16UC3) disp.convertTo(disp, CV_8U, 0.00390625);
   images.emplace_back(disp);
}

void AppUtils::displayDebugOutput(ApplicationState appState)
{
   hconcat(images, debugOutput);
   if (debugOutput.cols > 0 && debugOutput.rows > 0 && !debugOutput.empty())
   {
      namedWindow("DebugOutput", WINDOW_NORMAL);
      resizeWindow("DebugOutput", (int) (debugOutput.cols * appState.DISPLAY_WINDOW_SIZE), (int) (debugOutput.rows * appState.DISPLAY_WINDOW_SIZE));
      imshow("DebugOutput", debugOutput);
      waitKey(1);
   }
   images.clear();
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

