//
// Created by quantum on 2/16/21.
//

#include "AppUtils.h"

void AppUtils::getFileNames(string dirName, vector<string>& files)
{
   /* Get the sorted list of file names for planar regions at different frames. */
   if (auto dir = opendir(dirName.c_str()))
   {
      while (auto f = readdir(dir))
      {
         //         cout << f->d_name << endl;
         if (!f->d_name || f->d_name[0] == '.')
            continue;
         files.emplace_back(f->d_name);
      }
      closedir(dir);
   }
   sort(files.begin(), files.end());
}

void AppUtils::write_regions(vector<shared_ptr<PlanarRegion>> regions, string fileName)
{
   ofstream file;
   file.open(fileName, fstream::in | fstream::out | fstream::app);
   file << "NumRegions:" << regions.size() << endl;
   for (shared_ptr<PlanarRegion> region : regions)
   {
      region->writeToFile(file);
   }
   file.close();
   cout << "Writing Regions to:"
        << fileName << endl;
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
   write_regions(regions, projectPath + "/Extras/Regions/" + string(4 - to_string(0).length(), '0').append(to_string(0)) + ".txt");
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

