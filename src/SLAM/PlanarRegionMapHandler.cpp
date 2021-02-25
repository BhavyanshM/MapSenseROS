//
// Created by quantum on 2/8/21.
//

#include "PlanarRegionMapHandler.h"

void PlanarRegionMapHandler::registerRegions(vector<shared_ptr<PlanarRegion>> latestRegions)
{
   for (int i = 0; i < regions.size(); i++)
   {
      for (int j = 0; j < latestRegions.size(); j++)
      {
         Vector3f prevCenter = regions[i]->getCentroid();
         Vector3f curCenter = latestRegions[j]->getCentroid();
         Vector3f prevNormal = regions[i]->getNormal();
         Vector3f curNormal = latestRegions[j]->getNormal();
         float perpDist = fabs((prevCenter - curCenter).dot(curNormal)) + fabs((curCenter - prevCenter).dot(prevNormal));
         float angularDiff = fabs(prevNormal.dot(curNormal));
         //            ROS_INFO("DIFF:%.4lf", diff);
         if (perpDist < 0.1 && angularDiff > 0.7)
         {
            latestRegions[j]->setId(regions[i]->getId());
            break;
         }
      }
   }
}

void PlanarRegionMapHandler::getFileNames(string dirName)
{
   /* Get the sorted list of file names for planar regions at different frames. */
   this->directory = dirName;
   if (auto dir = opendir(dirName.c_str()))
   {
      while (auto f = readdir(dir))
      {
         // cout << f->d_name << endl;
         if (!f->d_name || f->d_name[0] == '.')
            continue;
         files.emplace_back(f->d_name);
      }
      closedir(dir);
   }
   sort(files.begin(), files.end());
}

Vector3f getVec3f(string csv)
{
   vector<string> CSVSubStrings;
   stringstream csvStream(csv);
   string csvStr;
   while (getline(csvStream, csvStr, ','))
   {
      CSVSubStrings.push_back(csvStr);
   }
   cout << "Vector:" << Vector3f(stof(CSVSubStrings[0]), stof(CSVSubStrings[1]), stof(CSVSubStrings[2])) << endl;
   return Vector3f(stof(CSVSubStrings[0]), stof(CSVSubStrings[1]), stof(CSVSubStrings[2]));
}

void PlanarRegionMapHandler::loadRegion(int frameId, shared_ptr<PlanarRegion> region)
{
   /* Generate planar region objects from the sorted list of files. */
   string regionText;
   ifstream regionFile(directory + files[frameId]);
   while (getline(regionFile, regionText))
   {
      vector<string> subStrings;
      stringstream ss(regionText);
      string str;
      while (getline(ss, str, ':'))
      {
         subStrings.push_back(str);
      }
      if (subStrings[0] == "RegionID" && subStrings.size() > 1)
      {
         cout << "RegionID:" << subStrings[1] << endl;
         region->setId(stoi(subStrings[1]));
      } else if (subStrings[0] == "Center" && subStrings.size() > 1)
      {
         cout << "Center:" << subStrings[1] << endl;
         region->setCenter(getVec3f(subStrings[1]));
      } else if (subStrings[0] == "Normal" && subStrings.size() > 1)
      {
         cout << "Normal:" << subStrings[1] << endl;
         region->setNormal(getVec3f(subStrings[1]));
      } else if (subStrings[0] == "NumPatches" && subStrings.size() > 1)
      {
         int length = stoi(subStrings[1]);
         for (int i = 0; i < length; i++)
         {
            getline(regionFile, regionText);
            cout << "RegionPoint:" << regionText << endl;
            region->insertBoundaryVertex(getVec3f(regionText));
         }
      }
   }
}




